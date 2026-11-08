/**************************************************************************/
/*  body_contacts_solver.cpp                                              */
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

#include "body_contacts_solver.h"

#include "core/object/class_db.h"
#include "scene/3d/skeleton_3d.h"

Skeleton3D *BodyContactsSolver::_skeleton() const {
	return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(skeleton_id));
}

void BodyContactsSolver::set_tuning(const Dictionary &p_tuning) {
	sections = p_tuning.get("sections", sections);
	sectors = p_tuning.get("sectors", sectors);
	profile_from = p_tuning.get("profile_from", profile_from);
	profile_to = p_tuning.get("profile_to", profile_to);
	reach_share = p_tuning.get("reach_share", reach_share);
	lean_reach = p_tuning.get("lean_reach", lean_reach);
	max_lean = p_tuning.get("max_lean", max_lean);
	forearm_samples = p_tuning.get("forearm_samples", forearm_samples);
	forearm_tolerance = p_tuning.get("forearm_tolerance", forearm_tolerance);
	tilt_step = p_tuning.get("tilt_step", tilt_step);
	max_tilt = p_tuning.get("max_tilt", max_tilt);
	near_straight = p_tuning.get("near_straight", near_straight);
	near_folded = p_tuning.get("near_folded", near_folded);
	pole_out = p_tuning.get("pole_out", pole_out);
}

void BodyContactsSolver::set_skeleton(Skeleton3D *p_skeleton) {
	skeleton_id = p_skeleton != nullptr ? p_skeleton->get_instance_id() : ObjectID();
}

void BodyContactsSolver::set_bone_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	bones.resize(p_count);
	for (Bone &bone : bones) {
		bone = Bone();
	}
	carries.resize(p_count);
	carried_on.resize(p_count);
	for (uint32_t &on : carried_on) {
		on = 0;
	}
}

void BodyContactsSolver::set_bone(int p_bone, int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest_inverse) {
	ERR_FAIL_INDEX(p_bone, (int)bones.size());
	bones[p_bone].index = p_index;
	bones[p_bone].rest_turn = p_rest_turn;
	bones[p_bone].skinned_rest_inverse = p_skinned_rest_inverse;
}

void BodyContactsSolver::set_points(const PackedInt32Array &p_point_triangles, const PackedVector3Array &p_point_weights,
		const PackedInt32Array &p_corners, const PackedVector3Array &p_positions, const PackedInt32Array &p_bones,
		const PackedFloat32Array &p_weights, const PackedVector3Array &p_normals, int p_slots) {
	ERR_FAIL_COND(p_slots <= 0);
	point_triangles = p_point_triangles;
	point_weights = p_point_weights;
	corners = p_corners;
	positions = p_positions;
	vertex_bones = p_bones;
	vertex_weights = p_weights;
	normals = p_normals;
	slots = p_slots;
}

void BodyContactsSolver::set_torso(const Vector3 &p_from, const Vector3 &p_axis, const Vector3 &p_front, const PackedFloat32Array &p_profile,
		int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest_inverse) {
	torso.set(p_from, p_axis, p_front, p_profile);
	torso_bone.index = p_index;
	torso_bone.rest_turn = p_rest_turn;
	torso_bone.skinned_rest_inverse = p_skinned_rest_inverse;
	has_torso = p_index >= 0;
}

void BodyContactsSolver::clear_torso() {
	has_torso = false;
}

void BodyContactsSolver::set_arm(int p_side, int p_shoulder, int p_elbow, int p_hand, real_t p_upper, real_t p_fore, real_t p_forearm_radius) {
	ERR_FAIL_INDEX(p_side, 2);
	Arm &arm = arms[p_side];
	arm.shoulder = p_shoulder;
	arm.elbow = p_elbow;
	arm.hand = p_hand;
	arm.upper = p_upper;
	arm.fore = p_fore;
	arm.forearm_radius = p_forearm_radius;
}

void BodyContactsSolver::set_lean_chain(const PackedInt32Array &p_chain) {
	lean_chain.resize(p_chain.size());
	for (int i = 0; i < p_chain.size(); i++) {
		lean_chain[i] = p_chain[i];
	}
}

void BodyContactsSolver::begin_tick() {
	tick++;
	if (tick == 0) {
		// Wrapped: every cached carry reads as current at 0.
		for (uint32_t &on : carried_on) {
			on = 0;
		}
		tick = 1;
	}
	for (Hand &hand : hands) {
		hand.has_before = false;
	}
}

Transform3D BodyContactsSolver::_carry_of(const Bone &p_bone, const Skeleton3D *p_skeleton) const {
	if (p_bone.index < 0 || p_bone.index >= p_skeleton->get_bone_count()) {
		return Transform3D();
	}
	const Transform3D pose = p_skeleton->get_bone_global_pose(p_bone.index);
	return Transform3D(pose.basis * p_bone.rest_turn, pose.origin) * p_bone.skinned_rest_inverse;
}

const Transform3D &BodyContactsSolver::_carry(int p_bone, const Skeleton3D *p_skeleton) {
	if (carried_on[p_bone] != tick) {
		carries[p_bone] = _carry_of(bones[p_bone], p_skeleton);
		carried_on[p_bone] = tick;
	}
	return carries[p_bone];
}

// BodyContactPoints.posed(): each corner skinned by its bones' carries,
// linear blend as the mesh is skinned; the normal turned by the same blend.
Vector3 BodyContactsSolver::_pose(int p_point, const Skeleton3D *p_skeleton, Vector3 &r_normal) {
	r_normal = Vector3();
	ERR_FAIL_INDEX_V(p_point, point_triangles.size(), Vector3());
	const int triangle = point_triangles[p_point];
	const Vector3 at = point_weights[p_point];
	const Vector3 normal = normals[triangle];
	Vector3 out;
	Vector3 turn;
	for (int k = 0; k < 3; k++) {
		const int corner = corners[triangle * 3 + k];
		const Vector3 rest = positions[corner];
		Vector3 skinned;
		Vector3 basis_sum;
		for (int slot = 0; slot < slots; slot++) {
			const int bone = vertex_bones[corner * slots + slot];
			const real_t w = vertex_weights[corner * slots + slot];
			if (bone < 0 || w <= 0.0 || bone >= (int)bones.size()) {
				continue;
			}
			const Transform3D &moved = _carry(bone, p_skeleton);
			skinned += moved.xform(rest) * w;
			basis_sum += moved.basis.xform(normal) * w;
		}
		out += skinned * at[k];
		turn += basis_sum * at[k];
	}
	r_normal = turn.normalized();
	return out;
}

Vector3 BodyContactsSolver::posed(int p_point) {
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, Vector3());
	return _pose(p_point, skeleton, posed_normal);
}

PackedVector3Array BodyContactsSolver::place(int p_side, int p_hand_point, int p_anchor_point) {
	PackedVector3Array out;
	ERR_FAIL_INDEX_V(p_side, 2, out);
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, out);
	Vector3 hand_normal;
	Vector3 anchor_normal;
	const Vector3 clip_point = _pose(p_hand_point, skeleton, hand_normal);
	const Vector3 anchor = _pose(p_anchor_point, skeleton, anchor_normal);
	hands[p_side].clip_point = clip_point;
	out.resize(3);
	out.set(0, clip_point);
	out.set(1, anchor);
	out.set(2, anchor_normal);
	return out;
}

bool BodyContactsSolver::_carried_by(const Skeleton3D *p_skeleton, int p_bone, int p_root) const {
	while (p_bone >= 0) {
		if (p_bone == p_root) {
			return true;
		}
		p_bone = p_skeleton->get_bone_parent(p_bone);
	}
	return false;
}

// BodyContacts.leaned().
Vector3 BodyContactsSolver::_leaned(const LocalVector<Vector3> &p_joints, const Vector3 &p_point, const Vector3 &p_axis, real_t p_angle) const {
	const real_t share = p_angle / p_joints.size();
	Vector3 at = p_joints[0];
	real_t turned = 0.0;
	for (uint32_t i = 0; i < p_joints.size(); i++) {
		turned += share;
		const Vector3 next = i == p_joints.size() - 1 ? p_point : p_joints[i + 1];
		at += (next - p_joints[i]).rotated(p_axis, turned);
	}
	return at;
}

// BodyContacts.lean_needed(): INF when no lean up to max_lean does.
real_t BodyContactsSolver::_lean_needed(const Skeleton3D *p_skeleton, const Vector3 &p_shoulder, const Vector3 &p_target, real_t p_want,
		Vector3 &r_axis) const {
	const Vector3 base = p_skeleton->get_bone_global_pose(lean_chain[0]).origin;
	Vector3 axis = (p_shoulder - base).cross(p_target - base);
	r_axis = Vector3();
	if (axis.length_squared() < 1e-12) {
		return Math::INF;
	}
	axis.normalize();
	r_axis = axis;
	LocalVector<Vector3> joints;
	joints.resize(lean_chain.size());
	for (uint32_t i = 0; i < lean_chain.size(); i++) {
		joints[i] = p_skeleton->get_bone_global_pose(lean_chain[i]).origin;
	}
	const real_t most = MIN(max_lean, (p_shoulder - base).angle_to(p_target - base));
	if (_leaned(joints, p_shoulder, axis, most).distance_to(p_target) > p_want) {
		return Math::INF;
	}
	real_t lo = 0.0;
	real_t hi = most;
	for (int i = 0; i < 24; i++) {
		const real_t mid = (lo + hi) * 0.5;
		if (_leaned(joints, p_shoulder, axis, mid).distance_to(p_target) > p_want) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	return hi;
}

// BodyContacts._lean()'s per hand part: the target is the clip's hand moved
// rigidly by `p_miss`; the lean its contact needs at full weight, or 0.
real_t BodyContactsSolver::aim(int p_side, const Vector3 &p_miss, int p_anchor_bone) {
	ERR_FAIL_INDEX_V(p_side, 2, 0.0);
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL_V(skeleton, 0.0);
	Hand &hand = hands[p_side];
	const Arm &arm = arms[p_side];
	hand.lean_axis = Vector3();
	const Transform3D clip_hand = skeleton->get_bone_global_pose(arm.hand);
	hand.local_target = Transform3D(clip_hand.basis, clip_hand.origin + p_miss);
	hand.point_in_hand = clip_hand.affine_inverse().xform(hand.clip_point);
	hand.anchor_bone = p_anchor_bone;
	hand.has_before = p_anchor_bone >= 0;
	if (hand.has_before) {
		hand.before = skeleton->get_bone_global_pose(p_anchor_bone);
	}
	const Vector3 shoulder = skeleton->get_bone_global_pose(arm.shoulder).origin;
	const real_t reach = arm.upper + arm.fore;
	const real_t away = hand.local_target.origin.distance_to(shoulder);
	hand.reachable = away <= reach * reach_share;
	if (away <= reach * lean_reach || lean_chain.is_empty() || p_anchor_bone < 0 || _carried_by(skeleton, p_anchor_bone, lean_chain[0])) {
		return 0.0;
	}
	Vector3 axis;
	const real_t need = _lean_needed(skeleton, shoulder, hand.local_target.origin, reach * lean_reach, axis);
	if (need > max_lean) {
		return 0.0;
	}
	hand.reachable = true;
	hand.lean_axis = axis;
	return need;
}

// BodyContacts.apply_lean(), then each aimed target carried with its
// anchor's bone through it.
void BodyContactsSolver::lean(const Vector3 &p_axis, real_t p_angle) {
	Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL(skeleton);
	if (lean_chain.is_empty()) {
		return;
	}
	const Basis share(p_axis, p_angle / lean_chain.size());
	LocalVector<Basis> olds;
	olds.resize(lean_chain.size());
	for (uint32_t i = 0; i < lean_chain.size(); i++) {
		olds[i] = skeleton->get_bone_global_pose(lean_chain[i]).basis.orthonormalized();
	}
	const Basis parent = skeleton->get_bone_global_pose(skeleton->get_bone_parent(lean_chain[0])).basis.orthonormalized();
	for (uint32_t i = 0; i < lean_chain.size(); i++) {
		const Basis &parent_basis = i == 0 ? parent : olds[i - 1];
		const Basis local = parent_basis.inverse() * share * olds[i];
		skeleton->set_bone_pose_rotation(lean_chain[i], local.get_rotation_quaternion());
	}
	for (Hand &hand : hands) {
		if (!hand.has_before) {
			continue;
		}
		const Transform3D moved = skeleton->get_bone_global_pose(hand.anchor_bone) * hand.before.affine_inverse();
		hand.local_target = moved * hand.local_target;
	}
}

// BodyContacts.elbow_circle().
BodyContactsSolver::Circle BodyContactsSolver::_elbow_circle(const Vector3 &p_shoulder, const Vector3 &p_elbow, const Vector3 &p_target,
		real_t p_upper, real_t p_fore) const {
	Circle circle;
	const Vector3 line = p_target - p_shoulder;
	const real_t d = CLAMP(line.length(), Math::abs(p_upper - p_fore) + (real_t)1e-4, p_upper + p_fore - (real_t)1e-4);
	if (line.length_squared() < 1e-10) {
		return circle;
	}
	const Vector3 n = line.normalized();
	const real_t x = (p_upper * p_upper - p_fore * p_fore + d * d) / (2.0 * d);
	const real_t r = Math::sqrt(MAX(p_upper * p_upper - x * x, (real_t)0.0));
	Vector3 off = p_elbow - p_shoulder;
	off -= n * off.dot(n);
	if (off.length_squared() < 1e-10) {
		return circle;
	}
	circle.valid = true;
	circle.centre = p_shoulder + n * x;
	circle.radius = r;
	circle.u = off.normalized();
	circle.v = n.cross(circle.u);
	return circle;
}

// BodyContacts.forearm_sink_of().
real_t BodyContactsSolver::_forearm_sink(real_t p_radius, const Transform3D &p_back, const Vector3 &p_elbow, const Vector3 &p_wrist) const {
	real_t worst = -Math::INF;
	for (int k = 0; k <= forearm_samples; k++) {
		const Vector3 p = p_back.xform(p_elbow.lerp(p_wrist, (real_t)k / forearm_samples));
		const Vector3 polar = torso.polar(p);
		if (polar.x < 0.0 || polar.x > 1.0) {
			continue;
		}
		worst = MAX(worst, p_radius - (polar.z - torso.skin_at(polar.x, polar.y, sections, sectors, profile_from, profile_to)));
	}
	return worst;
}

// BodyContacts.pole_of().
void BodyContactsSolver::_pole(int p_side, const Skeleton3D *p_skeleton) {
	Hand &hand = hands[p_side];
	const Arm &arm = arms[p_side];
	hand.has_pole = false;
	const Vector3 shoulder = p_skeleton->get_bone_global_pose(arm.shoulder).origin;
	const Vector3 elbow = p_skeleton->get_bone_global_pose(arm.elbow).origin;
	const Vector3 wrist = p_skeleton->get_bone_global_pose(arm.hand).origin;
	const real_t bend = (shoulder - elbow).angle_to(wrist - elbow);
	if (bend < near_straight && bend > near_folded) {
		return;
	}
	const Vector3 line = wrist - shoulder;
	const Vector3 along = shoulder + line * CLAMP((elbow - shoulder).dot(line) / MAX(line.length_squared(), (real_t)1e-8), (real_t)0.0, (real_t)1.0);
	const Vector3 out = elbow - along;
	if (out.length_squared() < 1e-8) {
		return;
	}
	hand.has_pole = true;
	hand.pole = elbow + out.normalized() * pole_out;
}

// BodyContacts._clear_forearm(): the least turn of the target about its
// contact point (tilt_step, up to max_tilt) that keeps the forearm out of the
// torso, then the elbow's pole.
void BodyContactsSolver::clear_forearm(int p_side) {
	ERR_FAIL_INDEX(p_side, 2);
	const Skeleton3D *skeleton = _skeleton();
	ERR_FAIL_NULL(skeleton);
	Hand &hand = hands[p_side];
	const Arm &arm = arms[p_side];
	hand.tilt = 0.0;
	hand.forearm_sink = 0.0;
	hand.clip_forearm_sink = 0.0;
	if (has_torso && arm.forearm_radius >= 0.0) {
		const real_t radius = arm.forearm_radius;
		// Read after the lean, which moved the chest.
		const Transform3D carry = _carry_of(torso_bone, skeleton);
		const Transform3D back = carry.affine_inverse();
		const Vector3 shoulder = skeleton->get_bone_global_pose(arm.shoulder).origin;
		const Vector3 elbow = skeleton->get_bone_global_pose(arm.elbow).origin;
		const Vector3 wrist = skeleton->get_bone_global_pose(arm.hand).origin;
		hand.clip_forearm_sink = _forearm_sink(radius, back, elbow, wrist);
		const real_t allowed = MAX(hand.clip_forearm_sink, (real_t)0.0) + forearm_tolerance;
		const Circle circle = _elbow_circle(shoulder, elbow, hand.local_target.origin, arm.upper, arm.fore);
		if (circle.valid) {
			hand.forearm_sink = _forearm_sink(radius, back, circle.centre + circle.u * circle.radius, hand.local_target.origin);
			if (hand.forearm_sink > allowed) {
				const Vector3 contact = hand.local_target.xform(hand.point_in_hand);
				const Vector3 at = hand.local_target.origin;
				const Vector3 rest_at = back.xform(at);
				const Vector3 polar = torso.polar(rest_at);
				const Vector3 outward = carry.basis.xform(rest_at - (torso.from + torso.axis * polar.x));
				Vector3 turn_axis = (at - contact).cross(outward);
				if (turn_axis.length_squared() > 1e-12) {
					turn_axis.normalize();
					Transform3D best = hand.local_target;
					real_t angle = tilt_step;
					while (angle <= max_tilt + 1e-4) {
						const Basis turn(turn_axis, angle);
						const Transform3D turned(turn * hand.local_target.basis, contact + turn.xform(at - contact));
						const Circle turned_circle = _elbow_circle(shoulder, elbow, turned.origin, arm.upper, arm.fore);
						if (turned_circle.valid) {
							const real_t sink = _forearm_sink(radius, back, turned_circle.centre + turned_circle.u * turned_circle.radius,
									turned.origin);
							if (sink < hand.forearm_sink) {
								hand.forearm_sink = sink;
								hand.tilt = angle;
								best = turned;
							}
							if (sink <= allowed) {
								break;
							}
						}
						angle += tilt_step;
					}
					hand.local_target = best;
				}
			}
		}
	}
	_pole(p_side, skeleton);
}

Transform3D BodyContactsSolver::get_local_target(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, Transform3D());
	return hands[p_side].local_target;
}

void BodyContactsSolver::set_local_target(int p_side, const Transform3D &p_target) {
	ERR_FAIL_INDEX(p_side, 2);
	hands[p_side].local_target = p_target;
}

bool BodyContactsSolver::is_reachable(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, true);
	return hands[p_side].reachable;
}

Vector3 BodyContactsSolver::get_lean_axis(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, Vector3());
	return hands[p_side].lean_axis;
}

Vector3 BodyContactsSolver::get_point_in_hand(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, Vector3());
	return hands[p_side].point_in_hand;
}

real_t BodyContactsSolver::get_tilt(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, 0.0);
	return hands[p_side].tilt;
}

real_t BodyContactsSolver::get_forearm_sink(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, 0.0);
	return hands[p_side].forearm_sink;
}

real_t BodyContactsSolver::get_clip_forearm_sink(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, 0.0);
	return hands[p_side].clip_forearm_sink;
}

bool BodyContactsSolver::has_pole(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, false);
	return hands[p_side].has_pole;
}

Vector3 BodyContactsSolver::get_pole(int p_side) const {
	ERR_FAIL_INDEX_V(p_side, 2, Vector3());
	return hands[p_side].pole;
}

void BodyContactsSolver::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_tuning", "tuning"), &BodyContactsSolver::set_tuning);
	ClassDB::bind_method(D_METHOD("set_skeleton", "skeleton"), &BodyContactsSolver::set_skeleton);
	ClassDB::bind_method(D_METHOD("set_bone_count", "count"), &BodyContactsSolver::set_bone_count);
	ClassDB::bind_method(D_METHOD("set_bone", "bone", "index", "rest_turn", "skinned_rest_inverse"), &BodyContactsSolver::set_bone);
	ClassDB::bind_method(D_METHOD("set_points", "point_triangles", "point_weights", "corners", "positions", "bones", "weights", "normals", "slots"),
			&BodyContactsSolver::set_points);
	ClassDB::bind_method(D_METHOD("set_torso", "from", "axis", "front", "profile", "index", "rest_turn", "skinned_rest_inverse"),
			&BodyContactsSolver::set_torso);
	ClassDB::bind_method(D_METHOD("clear_torso"), &BodyContactsSolver::clear_torso);
	ClassDB::bind_method(D_METHOD("set_arm", "side", "shoulder", "elbow", "hand", "upper", "fore", "forearm_radius"), &BodyContactsSolver::set_arm);
	ClassDB::bind_method(D_METHOD("set_lean_chain", "chain"), &BodyContactsSolver::set_lean_chain);
	ClassDB::bind_method(D_METHOD("begin_tick"), &BodyContactsSolver::begin_tick);
	ClassDB::bind_method(D_METHOD("posed", "point"), &BodyContactsSolver::posed);
	ClassDB::bind_method(D_METHOD("get_posed_normal"), &BodyContactsSolver::get_posed_normal);
	ClassDB::bind_method(D_METHOD("place", "side", "hand_point", "anchor_point"), &BodyContactsSolver::place);
	ClassDB::bind_method(D_METHOD("aim", "side", "miss", "anchor_bone"), &BodyContactsSolver::aim);
	ClassDB::bind_method(D_METHOD("lean", "axis", "angle"), &BodyContactsSolver::lean);
	ClassDB::bind_method(D_METHOD("clear_forearm", "side"), &BodyContactsSolver::clear_forearm);
	ClassDB::bind_method(D_METHOD("get_local_target", "side"), &BodyContactsSolver::get_local_target);
	ClassDB::bind_method(D_METHOD("set_local_target", "side", "target"), &BodyContactsSolver::set_local_target);
	ClassDB::bind_method(D_METHOD("is_reachable", "side"), &BodyContactsSolver::is_reachable);
	ClassDB::bind_method(D_METHOD("get_lean_axis", "side"), &BodyContactsSolver::get_lean_axis);
	ClassDB::bind_method(D_METHOD("get_point_in_hand", "side"), &BodyContactsSolver::get_point_in_hand);
	ClassDB::bind_method(D_METHOD("get_tilt", "side"), &BodyContactsSolver::get_tilt);
	ClassDB::bind_method(D_METHOD("get_forearm_sink", "side"), &BodyContactsSolver::get_forearm_sink);
	ClassDB::bind_method(D_METHOD("get_clip_forearm_sink", "side"), &BodyContactsSolver::get_clip_forearm_sink);
	ClassDB::bind_method(D_METHOD("has_pole", "side"), &BodyContactsSolver::has_pole);
	ClassDB::bind_method(D_METHOD("get_pole", "side"), &BodyContactsSolver::get_pole);
}
