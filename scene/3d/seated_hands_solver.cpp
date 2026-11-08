/**************************************************************************/
/*  seated_hands_solver.cpp                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "seated_hands_solver.h"

#include "core/object/class_db.h"
#include "scene/3d/skeleton_3d.h"

static real_t _smoothstep(real_t p_from, real_t p_to, real_t p_x) {
	if (Math::is_equal_approx(p_from, p_to)) {
		return p_x < p_from ? 0.0 : 1.0;
	}
	const real_t s = CLAMP((p_x - p_from) / (p_to - p_from), (real_t)0.0, (real_t)1.0);
	return s * s * (3.0 - 2.0 * s);
}

// The angle from p_from to p_to about p_axis, positive by the right hand.
static real_t _signed_angle(const Vector3 &p_from, const Vector3 &p_to, const Vector3 &p_axis) {
	const Vector3 a = p_from - p_axis * p_from.dot(p_axis);
	const Vector3 b = p_to - p_axis * p_to.dot(p_axis);
	if (a.length_squared() < 1e-10 || b.length_squared() < 1e-10) {
		return 0.0;
	}
	return Math::atan2(a.cross(b).dot(p_axis), a.dot(b));
}

// The palm's roll: p_normal from the thigh-facing direction square to the
// back of the hand's p_line.
static real_t _palm_roll(const Vector3 &p_normal, const Vector3 &p_line, const Vector3 &p_outward) {
	const Vector3 inward = -p_outward - p_line * (-p_outward).dot(p_line);
	return inward.length_squared() > 1e-8 ? p_normal.angle_to(inward.normalized()) : 0.0;
}

void SeatedHandsSolver::set_tuning(const Dictionary &p_tuning) {
	sections = p_tuning.get("sections", sections);
	sectors = p_tuning.get("sectors", sectors);
	profile_from = p_tuning.get("profile_from", profile_from);
	profile_to = p_tuning.get("profile_to", profile_to);
	contact_full = p_tuning.get("contact_full", contact_full);
	contact_none = p_tuning.get("contact_none", contact_none);
	max_side_angle = p_tuning.get("max_side_angle", max_side_angle);
	max_palm_roll = p_tuning.get("max_palm_roll", max_palm_roll);
	palm_clearance = p_tuning.get("palm_clearance", palm_clearance);
	resolve_passes = p_tuning.get("resolve_passes", resolve_passes);
	converged = p_tuning.get("converged", converged);
	max_extend = p_tuning.get("max_extend", max_extend);
	max_flex = p_tuning.get("max_flex", max_flex);
	max_drape = p_tuning.get("max_drape", max_drape);
	relaxed_bend = p_tuning.get("relaxed_bend", relaxed_bend);
	tip_share = p_tuning.get("tip_share", tip_share);
	newton_steps = p_tuning.get("newton_steps", newton_steps);
	contact_tolerance = p_tuning.get("contact_tolerance", contact_tolerance);
}

void SeatedHandsSolver::set_thigh(int p_side, const Vector3 &p_from, const Vector3 &p_axis, const Vector3 &p_front, const PackedFloat32Array &p_profile) {
	ERR_FAIL_INDEX(p_side, 2);
	thighs[p_side].set(p_from, p_axis, p_front, p_profile);
}

void SeatedHandsSolver::set_palm(int p_side, const Vector3 &p_wrist, const Vector3 &p_knuckle, const Vector3 &p_normal, const Vector3 &p_point,
		const PackedVector3Array &p_contacts, real_t p_finger_pad) {
	ERR_FAIL_INDEX(p_side, 2);
	Palm &palm = palms[p_side];
	palm.wrist = p_wrist;
	palm.knuckle = p_knuckle;
	palm.normal = p_normal;
	palm.point = p_point;
	palm.contacts.resize(p_contacts.size());
	for (int i = 0; i < p_contacts.size(); i++) {
		palm.contacts[i] = p_contacts[i];
	}
	palm.finger_pad = p_finger_pad;
}

void SeatedHandsSolver::set_skeleton(Skeleton3D *p_skeleton) {
	skeleton_id = p_skeleton ? p_skeleton->get_instance_id() : ObjectID();
}

void SeatedHandsSolver::set_bone(int p_side, bool p_hand, int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest, const Basis &p_to_general) {
	ERR_FAIL_INDEX(p_side, 2);
	Bone &bone = p_hand ? hand_bones[p_side] : thigh_bones[p_side];
	bone.index = p_index;
	bone.rest_turn = p_rest_turn;
	bone.skinned_rest = p_skinned_rest;
	bone.skinned_rest_inverse = p_skinned_rest.affine_inverse();
	bone.to_general = p_to_general;
}

void SeatedHandsSolver::set_finger_bones(int p_side, int p_hand, int p_middle, int p_index, int p_little, const PackedInt32Array &p_bones) {
	ERR_FAIL_INDEX(p_side, 2);
	ERR_FAIL_COND_MSG(p_bones.size() % 3 != 0, "Three bones a finger.");
	Fingers &f = fingers[p_side];
	f.hand = p_hand;
	f.middle = p_middle;
	f.index = p_index;
	f.little = p_little;
	f.bones.resize(p_bones.size());
	for (int i = 0; i < p_bones.size(); i++) {
		f.bones[i] = p_bones[i];
	}
	f.angles.clear();
	f.axes.clear();
	f.shown.clear();
}

Skeleton3D *SeatedHandsSolver::_skeleton() const {
	return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(skeleton_id));
}

Transform3D SeatedHandsSolver::_carry(const Bone &p_bone, const Skeleton3D *p_skeleton) const {
	const Transform3D pose = p_skeleton->get_bone_global_pose(p_bone.index);
	return Transform3D(pose.basis * p_bone.rest_turn, pose.origin) * p_bone.skinned_rest_inverse;
}

Vector3 SeatedHandsSolver::_polar(const Thigh &p_thigh, const Vector3 &p_point) const {
	return p_thigh.polar(p_point);
}

real_t SeatedHandsSolver::_skin_at(const Thigh &p_thigh, real_t p_t, real_t p_angle) const {
	return p_thigh.skin_at(p_t, p_angle, sections, sectors, profile_from, profile_to);
}

real_t SeatedHandsSolver::_gap(const Thigh &p_thigh, const Vector3 &p_point) const {
	const Vector3 p = _polar(p_thigh, p_point);
	return p.z - _skin_at(p_thigh, p.x, p.y);
}

Vector3 SeatedHandsSolver::_outward(const Thigh &p_thigh, const Vector3 &p_at) const {
	return (p_at - p_thigh.from - p_thigh.axis * _polar(p_thigh, p_at).x).normalized();
}

Transform3D SeatedHandsSolver::_rest_move(const Thigh &p_thigh, const Vector3 &p_wrist, const Vector3 &p_line, const Vector3 &p_normal,
		const Vector3 &p_point, const LocalVector<Vector3> *p_contacts, const Vector3 &p_outward, real_t p_roll) const {
	const Vector3 flat = (p_line - p_outward * p_line.dot(p_outward)).normalized();
	// Lay the back of the hand along the thigh...
	Quaternion turn(p_line, flat);
	// ...then roll it to p_roll, about that line, keeping its direction.
	const Vector3 turned_normal = turn.xform(p_normal);
	Vector3 inward = -p_outward - flat * (-p_outward).dot(flat);
	if (inward.length_squared() > 1e-8) {
		inward.normalize();
		const Vector3 about = turned_normal.cross(inward);
		if (about.length_squared() > 1e-10) {
			turn = Quaternion(about.normalized(), turned_normal.angle_to(inward) - p_roll) * turn;
		}
	}
	const Vector3 turned_point = p_wrist + turn.xform(p_point - p_wrist);
	// Then onto the skin: the clip's place along and round the thigh, within
	// the profile and max_side_angle of the top.
	const Vector3 polar = _polar(p_thigh, turned_point);
	const real_t t = CLAMP(polar.x, profile_from, profile_to);
	const real_t angle = CLAMP(polar.y, -max_side_angle, max_side_angle);
	const Vector3 around = p_thigh.front * Math::cos(angle) + p_thigh.side * Math::sin(angle);
	const Vector3 placed = p_thigh.from + p_thigh.axis * t + around * (_skin_at(p_thigh, t, angle) + palm_clearance);
	const Basis basis(turn);
	Transform3D result(basis, p_wrist - basis.xform(p_wrist) + (placed - turned_point));
	if (p_contacts == nullptr) {
		return result;
	}
	// None of the palm's contact points under the clearance either: a rolled
	// palm's edge is lower than its point.
	real_t lowest = Math::INF;
	Vector3 under;
	for (const Vector3 &contact : *p_contacts) {
		const real_t g = _gap(p_thigh, result.xform(contact));
		if (g < lowest) {
			lowest = g;
			under = contact;
		}
	}
	// Out along that contact's own radial, which its gap follows nearly 1:1;
	// the same contact once more, for the curve.
	for (int lift = 0; lift < 2; lift++) {
		if (lowest >= palm_clearance - 0.0005) {
			break;
		}
		result.origin += _outward(p_thigh, result.xform(under)) * (palm_clearance - lowest);
		lowest = _gap(p_thigh, result.xform(under));
	}
	return result;
}

void SeatedHandsSolver::_left_to_clip(const Transform3D &p_hand_carry, const Transform3D &p_hand_rest) {
	hand = p_hand_carry * p_hand_rest;
	ik_target = hand;
}

real_t SeatedHandsSolver::gap(int p_side, const Vector3 &p_point) const {
	ERR_FAIL_INDEX_V(p_side, 2, Math::INF);
	return _gap(thighs[p_side], p_point);
}

Transform3D SeatedHandsSolver::carry(int p_side, bool p_hand) const {
	ERR_FAIL_INDEX_V(p_side, 2, Transform3D());
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, Transform3D());
	return _carry(p_hand ? hand_bones[p_side] : thigh_bones[p_side], skeleton);
}

Transform3D SeatedHandsSolver::clip(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, Transform3D());
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, Transform3D());
	return _carry(thigh_bones[p_side], skeleton).affine_inverse() * _carry(hand_bones[p_side], skeleton);
}

void SeatedHandsSolver::solve_carried(int p_side, const Transform3D &p_thigh_carry, const Transform3D &p_hand_carry, const Transform3D &p_hand_rest,
		const Vector3 &p_warm) {
	ERR_FAIL_INDEX(p_side, 2);
	weight = 0.0;
	tilt = 0.0;
	roll = 0.0;
	move = Transform3D();
	outward = Vector3();
	const Thigh &thigh = thighs[p_side];
	const Palm &palm = palms[p_side];
	// Into the thigh's rest frame, where its profile reads.
	const Transform3D into_thigh = p_thigh_carry.affine_inverse() * p_hand_carry;
	const Vector3 wrist = into_thigh.xform(palm.wrist);
	const Vector3 knuckle = into_thigh.xform(palm.knuckle);
	const Vector3 point = into_thigh.xform(palm.point);
	const Vector3 normal = into_thigh.basis.xform(palm.normal).normalized();
	LocalVector<Vector3> contacts;
	contacts.resize(palm.contacts.size());
	for (uint32_t i = 0; i < palm.contacts.size(); i++) {
		contacts[i] = into_thigh.xform(palm.contacts[i]);
	}
	palm_gap = _gap(thigh, point);
	const Vector3 axis = thigh.axis.normalized();
	const Vector3 line = (knuckle - wrist).normalized();
	Vector3 out = _outward(thigh, wrist);
	tilt = Math::asin(CLAMP(line.dot(out), (real_t)-1.0, (real_t)1.0));
	const Vector3 flat = (line - out * line.dot(out)).normalized();
	// A hand pointing back up the thigh, toward the hip: left to the clip.
	if (flat.dot(axis) < 0.0) {
		_left_to_clip(p_hand_carry, p_hand_rest);
		return;
	}
	weight = 1.0 - _smoothstep(contact_full, contact_none, palm_gap);
	roll = _palm_roll(normal, line, out);
	// Turned against the outward where the wrist lands, in a few passes; the
	// contacts on the last only.
	const real_t capped = MIN(roll, max_palm_roll);
	int passes = resolve_passes;
	if (p_warm != Vector3()) {
		out = p_warm;
		passes = 1;
	}
	Vector3 landed = wrist;
	for (int i = 0; i < passes; i++) {
		const Vector3 at = _rest_move(thigh, wrist, line, normal, point, nullptr, out, capped).xform(wrist);
		out = _outward(thigh, at);
		if (i > 0 && at.distance_squared_to(landed) < converged * converged) {
			break;
		}
		landed = at;
	}
	move = _rest_move(thigh, wrist, line, normal, point, &contacts, out, capped);
	outward = out;
	hand = p_thigh_carry * move * into_thigh * p_hand_rest;
	ik_target = hand;
}

void SeatedHandsSolver::reapply_carried(int p_side, const Transform3D &p_thigh_carry, const Transform3D &p_hand_carry, const Transform3D &p_hand_rest,
		const Transform3D &p_move) {
	ERR_FAIL_INDEX(p_side, 2);
	weight = 0.0;
	tilt = 0.0;
	roll = 0.0;
	move = Transform3D();
	outward = Vector3();
	const Thigh &thigh = thighs[p_side];
	const Palm &palm = palms[p_side];
	const Transform3D into_thigh = p_thigh_carry.affine_inverse() * p_hand_carry;
	palm_gap = _gap(thigh, into_thigh.xform(palm.point));
	const Vector3 wrist = into_thigh.xform(palm.wrist);
	const Vector3 line = (into_thigh.xform(palm.knuckle) - wrist).normalized();
	const Vector3 out = _outward(thigh, wrist);
	if ((line - out * line.dot(out)).dot(thigh.axis) < 0.0) {
		_left_to_clip(p_hand_carry, p_hand_rest);
		return;
	}
	weight = 1.0 - _smoothstep(contact_full, contact_none, palm_gap);
	move = p_move;
	hand = p_thigh_carry * p_move * into_thigh * p_hand_rest;
	ik_target = hand;
}

void SeatedHandsSolver::solve(int p_side, bool p_full, const Transform3D &p_move, const Vector3 &p_warm) {
	ERR_FAIL_INDEX(p_side, 2);
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL(skeleton);
	const Bone &h = hand_bones[p_side];
	const Transform3D thigh_carry = _carry(thigh_bones[p_side], skeleton);
	const Transform3D hand_carry = _carry(h, skeleton);
	if (p_full) {
		solve_carried(p_side, thigh_carry, hand_carry, h.skinned_rest, p_warm);
	} else {
		reapply_carried(p_side, thigh_carry, hand_carry, h.skinned_rest, p_move);
	}
	// Into the animated skeleton's hand bone's frame.
	ik_target = Transform3D(hand.basis * h.to_general, hand.origin);
}

Vector2 SeatedHandsSolver::_off(const Thigh &p_thigh, const Vector3 &p_joint, const Vector3 &p_segment, const Vector3 &p_axis, real_t p_pad, real_t p_angle) const {
	const Vector3 arm = p_segment.rotated(p_axis, p_angle);
	const Vector3 end = p_joint + arm;
	const Vector3 polar = _polar(p_thigh, end);
	const Vector3 radial = end - p_thigh.from - p_thigh.axis * polar.x;
	return Vector2(polar.z - _skin_at(p_thigh, polar.x, polar.y) - p_pad,
			p_axis.cross(arm).dot(radial / MAX(polar.z, (real_t)1e-4)));
}

// The turn about p_axis at p_joint that sets the segment's end p_pad off the
// thigh's skin, within [p_low, p_high]; bracketed on the side the plan
// means, then Newton steps within the bracket, bisecting where one would
// leave it.
real_t SeatedHandsSolver::_contact_angle(const Thigh &p_thigh, const Vector3 &p_joint, const Vector3 &p_segment, const Vector3 &p_axis,
		real_t p_pad, real_t p_low, real_t p_high, real_t p_bend) const {
	const Vector2 now = _off(p_thigh, p_joint, p_segment, p_axis, p_pad, 0.0);
	if (Math::abs(now.x) < contact_tolerance) {
		return 0.0;
	}
	real_t a = 0.0;
	const real_t fa = now.x;
	real_t b;
	if (now.x < 0.0) {
		b = MIN(p_low, (real_t)0.0);
		if (b >= 0.0 || _off(p_thigh, p_joint, p_segment, p_axis, p_pad, b).x < 0.0) {
			return MIN(b, (real_t)0.0);
		}
	} else {
		b = MIN(p_high, MAX((real_t)0.0, max_drape - MAX(p_bend, (real_t)0.0)));
		if (b <= 0.0 || _off(p_thigh, p_joint, p_segment, p_axis, p_pad, b).x > 0.0) {
			// Over air even at the drape's limit: relaxed.
			const real_t relaxed = CLAMP(relaxed_bend - p_bend, p_low, p_high);
			return _off(p_thigh, p_joint, p_segment, p_axis, p_pad, relaxed).x >= 0.0 ? relaxed : MAX(b, (real_t)0.0);
		}
	}
	real_t angle = a;
	Vector2 step = now;
	for (int i = 0; i < newton_steps; i++) {
		real_t next = Math::abs(step.y) > 1e-6 ? angle - step.x / step.y : (a + b) * 0.5;
		if ((next - a) * (next - b) >= 0.0) {
			next = (a + b) * 0.5;
		}
		angle = next;
		step = _off(p_thigh, p_joint, p_segment, p_axis, p_pad, angle);
		if (Math::abs(step.x) < contact_tolerance) {
			return angle;
		}
		if ((step.x < 0.0) == (fa < 0.0)) {
			a = angle;
		} else {
			b = angle;
		}
	}
	return angle;
}

void SeatedHandsSolver::search_fingers(int p_side) {
	ERR_FAIL_INDEX(p_side, 2);
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL(skeleton);
	Fingers &f = fingers[p_side];
	ERR_FAIL_COND(f.hand < 0);
	const Thigh &thigh = thighs[p_side];
	const Palm &palm = palms[p_side];
	const Transform3D hand_pose = skeleton->get_bone_global_pose(f.hand);
	const Vector3 normal = _carry(hand_bones[p_side], skeleton).basis.xform(palm.normal).normalized();
	const Vector3 line = (skeleton->get_bone_global_pose(f.middle).origin - hand_pose.origin).normalized();
	// The knuckle line, signed so a positive turn curls toward the palm.
	Vector3 axis = (skeleton->get_bone_global_pose(f.index).origin - skeleton->get_bone_global_pose(f.little).origin).normalized();
	if (axis.cross(line).dot(normal) < 0.0) {
		axis = -axis;
	}
	const Transform3D to_thigh = _carry(thigh_bones[p_side], skeleton).affine_inverse();
	const Vector3 thigh_axis = to_thigh.basis.xform(axis).normalized();
	const real_t pad = palm.finger_pad + palm_clearance;
	f.angles.clear();
	f.axes.clear();
	for (uint32_t first = 0; first + 2 < f.bones.size(); first += 3) {
		Transform3D poses[3];
		for (int k = 0; k < 3; k++) {
			poses[k] = skeleton->get_bone_global_pose(f.bones[first + k]);
		}
		// The finger's joints and tip, as each turn found leaves them.
		const Vector3 along = poses[1].basis.inverse().xform(poses[2].origin - poses[1].origin);
		Vector3 points[4] = { poses[0].origin, poses[1].origin, poses[2].origin,
			poses[2].origin + poses[2].basis.xform(along) * tip_share };
		Vector3 parent_point = hand_pose.origin;
		for (int k = 0; k < 3; k++) {
			const Vector3 segment = points[k + 1] - points[k];
			const real_t bend = _signed_angle((points[k] - parent_point).normalized(), segment.normalized(), axis);
			const real_t angle = _contact_angle(thigh, to_thigh.xform(points[k]), to_thigh.basis.xform(segment), thigh_axis,
					pad, -max_extend - bend, max_flex - bend, bend);
			f.angles.push_back(angle);
			f.axes.push_back(poses[k].basis.inverse().xform(axis).normalized());
			for (int m = k + 1; m < 4; m++) {
				points[m] = points[k] + (points[m] - points[k]).rotated(axis, angle);
			}
			parent_point = points[k];
		}
	}
}

void SeatedHandsSolver::ease_fingers(int p_side, real_t p_share, real_t p_limit) {
	ERR_FAIL_INDEX(p_side, 2);
	Fingers &f = fingers[p_side];
	if (f.shown.size() != f.angles.size()) {
		// The first search is shown at once.
		f.shown = f.angles;
		return;
	}
	for (uint32_t k = 0; k < f.shown.size(); k++) {
		f.shown[k] += CLAMP((f.angles[k] - f.shown[k]) * p_share, -p_limit, p_limit);
	}
}

real_t SeatedHandsSolver::lay_fingers(int p_side, real_t p_weight) {
	ERR_FAIL_INDEX_V(p_side, 2, 0.0);
	Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, 0.0);
	const Fingers &f = fingers[p_side];
	if (f.shown.size() != f.bones.size() || f.axes.size() != f.bones.size()) {
		return 0.0;
	}
	real_t turned = 0.0;
	for (uint32_t n = 0; n < f.bones.size(); n++) {
		const real_t angle = f.shown[n] * p_weight;
		if (Math::abs(angle) > 1e-5) {
			skeleton->set_bone_pose_rotation(f.bones[n], skeleton->get_bone_pose_rotation(f.bones[n]) * Quaternion(f.axes[n], angle));
			turned += Math::abs(angle);
		}
	}
	return turned;
}

void SeatedHandsSolver::clear_fingers(int p_side) {
	ERR_FAIL_INDEX(p_side, 2);
	fingers[p_side].angles.clear();
	fingers[p_side].axes.clear();
	fingers[p_side].shown.clear();
}

bool SeatedHandsSolver::has_fingers(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, false);
	return fingers[p_side].hand >= 0;
}

bool SeatedHandsSolver::has_finger_angles(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, false);
	return !fingers[p_side].angles.is_empty();
}

PackedFloat32Array SeatedHandsSolver::get_finger_angles(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, PackedFloat32Array());
	PackedFloat32Array out;
	for (const real_t angle : fingers[p_side].angles) {
		out.push_back(angle);
	}
	return out;
}

void SeatedHandsSolver::set_finger_angles(int p_side, const PackedFloat32Array &p_angles) {
	ERR_FAIL_INDEX(p_side, 2);
	Fingers &f = fingers[p_side];
	f.angles.resize(p_angles.size());
	for (int i = 0; i < p_angles.size(); i++) {
		f.angles[i] = p_angles[i];
	}
}

PackedFloat32Array SeatedHandsSolver::get_finger_shown(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, PackedFloat32Array());
	PackedFloat32Array out;
	for (const real_t angle : fingers[p_side].shown) {
		out.push_back(angle);
	}
	return out;
}

void SeatedHandsSolver::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_tuning", "tuning"), &SeatedHandsSolver::set_tuning);
	ClassDB::bind_method(D_METHOD("set_thigh", "side", "from", "axis", "front", "profile"), &SeatedHandsSolver::set_thigh);
	ClassDB::bind_method(D_METHOD("set_palm", "side", "wrist", "knuckle", "normal", "point", "contacts", "finger_pad"), &SeatedHandsSolver::set_palm);
	ClassDB::bind_method(D_METHOD("set_skeleton", "skeleton"), &SeatedHandsSolver::set_skeleton);
	ClassDB::bind_method(D_METHOD("set_bone", "side", "hand", "index", "rest_turn", "skinned_rest", "to_general"), &SeatedHandsSolver::set_bone);
	ClassDB::bind_method(D_METHOD("set_finger_bones", "side", "hand", "middle", "index", "little", "bones"), &SeatedHandsSolver::set_finger_bones);
	ClassDB::bind_method(D_METHOD("gap", "side", "point"), &SeatedHandsSolver::gap);
	ClassDB::bind_method(D_METHOD("carry", "side", "hand"), &SeatedHandsSolver::carry);
	ClassDB::bind_method(D_METHOD("clip", "side"), &SeatedHandsSolver::clip);
	ClassDB::bind_method(D_METHOD("solve_carried", "side", "thigh_carry", "hand_carry", "hand_rest", "warm"), &SeatedHandsSolver::solve_carried);
	ClassDB::bind_method(D_METHOD("reapply_carried", "side", "thigh_carry", "hand_carry", "hand_rest", "move"), &SeatedHandsSolver::reapply_carried);
	ClassDB::bind_method(D_METHOD("solve", "side", "full", "move", "warm"), &SeatedHandsSolver::solve);
	ClassDB::bind_method(D_METHOD("get_weight"), &SeatedHandsSolver::get_weight);
	ClassDB::bind_method(D_METHOD("get_palm_gap"), &SeatedHandsSolver::get_palm_gap);
	ClassDB::bind_method(D_METHOD("get_tilt"), &SeatedHandsSolver::get_tilt);
	ClassDB::bind_method(D_METHOD("get_roll"), &SeatedHandsSolver::get_roll);
	ClassDB::bind_method(D_METHOD("get_hand"), &SeatedHandsSolver::get_hand);
	ClassDB::bind_method(D_METHOD("get_ik_target"), &SeatedHandsSolver::get_ik_target);
	ClassDB::bind_method(D_METHOD("get_move"), &SeatedHandsSolver::get_move);
	ClassDB::bind_method(D_METHOD("get_outward"), &SeatedHandsSolver::get_outward);
	ClassDB::bind_method(D_METHOD("search_fingers", "side"), &SeatedHandsSolver::search_fingers);
	ClassDB::bind_method(D_METHOD("ease_fingers", "side", "share", "limit"), &SeatedHandsSolver::ease_fingers);
	ClassDB::bind_method(D_METHOD("lay_fingers", "side", "weight"), &SeatedHandsSolver::lay_fingers);
	ClassDB::bind_method(D_METHOD("clear_fingers", "side"), &SeatedHandsSolver::clear_fingers);
	ClassDB::bind_method(D_METHOD("has_fingers", "side"), &SeatedHandsSolver::has_fingers);
	ClassDB::bind_method(D_METHOD("has_finger_angles", "side"), &SeatedHandsSolver::has_finger_angles);
	ClassDB::bind_method(D_METHOD("get_finger_angles", "side"), &SeatedHandsSolver::get_finger_angles);
	ClassDB::bind_method(D_METHOD("set_finger_angles", "side", "angles"), &SeatedHandsSolver::set_finger_angles);
	ClassDB::bind_method(D_METHOD("get_finger_shown", "side"), &SeatedHandsSolver::get_finger_shown);
}
