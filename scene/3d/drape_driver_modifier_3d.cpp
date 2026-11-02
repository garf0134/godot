/**************************************************************************/
/*  drape_driver_modifier_3d.cpp                                          */
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


#include "drape_driver_modifier_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/3d/drape_rings_modifier_3d.h"
#include "scene/3d/spring_bone_simulator_3d.h"
#include "scene/3d/visual_instance_3d.h"
#include "scene/main/scene_tree.h"
#include "servers/rendering/rendering_server.h"

// How far inward a swing carries a column (the cosine of its push with the
// column's outward) before it is followed wholly reversed.
static const float INWARD_BAND = 0.5f;
// compute_pelvis()'s packing, DrapeSeatedTarget.PACK_STEPS.
static const float PELVIS_STEPS = 4095.0f;

static Quaternion _quaternion_of(const Vector4 &p_v) {
	return Quaternion(p_v.x, p_v.y, p_v.z, p_v.w);
}

void DrapeDriverModifier3D::set_layout(const PackedInt32Array &p_thighs, const PackedVector4Array &p_thigh_rests, const PackedVector3Array &p_thigh_along,
		const PackedInt32Array &p_tops, const PackedVector4Array &p_rests, const PackedVector2Array &p_shares, const PackedVector2Array &p_weights,
		const PackedVector3Array &p_downs, const PackedVector3Array &p_outs, const PackedVector4Array &p_seat_turns, const PackedFloat32Array &p_laps,
		const Vector3 &p_up) {
	clear_layout();
	ERR_FAIL_COND_MSG(p_thighs.size() != 2 || p_thigh_rests.size() != 2 || p_thigh_along.size() != 2, "Two thighs, each with a rest and a direction.");
	const int n = p_tops.size();
	ERR_FAIL_COND_MSG(p_rests.size() != n || p_shares.size() != n || p_weights.size() != n || p_downs.size() != n || p_outs.size() != n || p_seat_turns.size() != n || p_laps.size() != n,
			"One rest, share, weight, down, out, seat turn and lap per column.");
	for (int k = 0; k < 2; k++) {
		thighs.push_back(p_thighs[k]);
		thigh_rests.push_back(_quaternion_of(p_thigh_rests[k]));
		thigh_along.push_back(p_thigh_along[k]);
	}
	for (int c = 0; c < n; c++) {
		tops.push_back(p_tops[c]);
		rests.push_back(_quaternion_of(p_rests[c]));
		downs.push_back(p_downs[c]);
		outs.push_back(p_outs[c]);
		seat_turns.push_back(_quaternion_of(p_seat_turns[c]));
		laps.push_back(p_laps[c]);
	}
	shares = p_shares;
	weights = p_weights;
	applied = p_weights;
	up = p_up;
}

void DrapeDriverModifier3D::set_seat_turns(const PackedVector4Array &p_seat_turns, const PackedFloat32Array &p_presses) {
	ERR_FAIL_COND_MSG(p_seat_turns.size() != (int)seat_turns.size() || (p_presses.size() != 0 && p_presses.size() != p_seat_turns.size()),
			"set_seat_turns() needs a turn per column of the layout, and a press per column or none.");
	for (int c = 0; c < p_seat_turns.size(); c++) {
		seat_turns[c] = _quaternion_of(p_seat_turns[c]);
	}
	presses.clear();
	for (int c = 0; c < p_presses.size(); c++) {
		presses.push_back(p_presses[c]);
	}
}

void DrapeDriverModifier3D::set_pose_targets(const PackedInt32Array &p_bones, const PackedVector4Array &p_rests, const PackedVector4Array &p_targets,
		const PackedFloat32Array &p_legs, const PackedInt32Array &p_knees, const PackedVector4Array &p_knee_rests,
		const PackedVector3Array &p_shin_along, const Vector4 &p_hips_rest) {
	clear_pose_targets();
	const int n = p_bones.size();
	ERR_FAIL_COND_MSG(p_rests.size() != n, "One rest per chain bone.");
	ERR_FAIL_COND_MSG(p_legs.size() % 4 != 0 || (n == 0 ? p_targets.size() != 0 : p_targets.size() != n * (p_legs.size() / 4)),
			"Four leg angles per variant, and a target per chain bone per variant.");
	ERR_FAIL_COND_MSG(p_knees.size() != 2 || p_knee_rests.size() != 2 || p_shin_along.size() != 2, "Two knees, each with a rest and a direction.");
	for (int i = 0; i < n; i++) {
		pose_bones.push_back(p_bones[i]);
		pose_rests.push_back(_quaternion_of(p_rests[i]));
	}
	for (int i = 0; i < p_targets.size(); i++) {
		pose_targets.push_back(_quaternion_of(p_targets[i]));
	}
	for (int i = 0; i < p_legs.size(); i++) {
		pose_legs.push_back(p_legs[i]);
	}
	pose_variants = n > 0 ? p_legs.size() / 4 : 0;
	for (int k = 0; k < 2; k++) {
		knees[k] = p_knees[k];
		knee_rests[k] = _quaternion_of(p_knee_rests[k]);
		shin_along[k] = p_shin_along[k];
	}
	hips_rest = _quaternion_of(p_hips_rest);
	pose_tops.resize(tops.size());
	pose_is_top.resize(n);
	for (int i = 0; i < n; i++) {
		pose_is_top[i] = 0;
	}
	for (uint32_t c = 0; c < tops.size(); c++) {
		pose_tops[c] = -1;
		for (int i = 0; i < n; i++) {
			if (pose_bones[i] == tops[c]) {
				pose_tops[c] = i;
				pose_is_top[i] = 1;
				break;
			}
		}
	}
}

void DrapeDriverModifier3D::clear_pose_targets() {
	pose_bones.clear();
	pose_rests.clear();
	pose_targets.clear();
	pose_legs.clear();
	pose_tops.clear();
	pose_is_top.clear();
	pose_variants = 0;
	knees[0] = -1;
	knees[1] = -1;
	pose_variant = -1;
	pose_weight = 0.0;
	legs.clear();
}

void DrapeDriverModifier3D::set_pose_tuning(double p_from, double p_stray) {
	pose_from = p_from;
	pose_stray = p_stray;
}

double DrapeDriverModifier3D::side_angle(const Vector3 &p_v) {
	return Math::atan2((double)p_v.z, (double)-p_v.y);
}

double DrapeDriverModifier3D::weigh_pose(const PackedFloat32Array &p_legs, const PackedFloat32Array &p_variant, double p_from, double p_stray) {
	ERR_FAIL_COND_V(p_legs.size() < p_variant.size(), 0.0);
	double length_squared = 0.0;
	double along = 0.0;
	for (int k = 0; k < p_variant.size(); k++) {
		length_squared += p_variant[k] * p_variant[k];
		along += p_legs[k] * p_variant[k];
	}
	if (length_squared < 1e-8) {
		return 0.0;
	}
	const double t = along / length_squared;
	double stray = 0.0;
	for (int k = 0; k < p_variant.size(); k++) {
		const double off = p_legs[k] - t * p_variant[k];
		stray += off * off;
	}
	stray = Math::sqrt(stray / length_squared);
	return Math::smoothstep(p_from, 1.0, t) * (1.0 - Math::smoothstep(p_stray, p_stray * 2.0, stray));
}

void DrapeDriverModifier3D::_measure_legs(Skeleton3D *p_skeleton) {
	legs.resize(4);
	float *out = legs.ptrw();
	for (int k = 0; k < 2; k++) {
		const Vector3 thigh = hips_rest.xform(p_skeleton->get_bone_pose_rotation(thighs[k]).xform(thigh_along[k]));
		const Vector3 thigh_rest = hips_rest.xform(thigh_rests[k].xform(thigh_along[k]));
		out[k] = side_angle(thigh) - side_angle(thigh_rest);
		const Vector3 shin = p_skeleton->get_bone_pose_rotation(knees[k]).xform(shin_along[k]);
		const Vector3 shin_rest = knee_rests[k].xform(shin_along[k]);
		out[2 + k] = thigh_along[k].angle_to(shin) - thigh_along[k].angle_to(shin_rest);
	}
}

void DrapeDriverModifier3D::_weigh_poses() {
	pose_variant = -1;
	pose_weight = 0.0;
	PackedFloat32Array variant;
	variant.resize(4);
	for (int v = 0; v < pose_variants; v++) {
		for (int k = 0; k < 4; k++) {
			variant.set(k, pose_legs[v * 4 + k]);
		}
		const double w = weigh_pose(legs, variant, pose_from, pose_stray);
		if (w > pose_weight) {
			pose_weight = w;
			pose_variant = v;
		}
	}
}

void DrapeDriverModifier3D::clear_layout() {
	clear_pose_targets();
	thighs.clear();
	thigh_rests.clear();
	thigh_along.clear();
	tops.clear();
	rests.clear();
	shares.clear();
	weights.clear();
	downs.clear();
	outs.clear();
	seat_turns.clear();
	laps.clear();
	applied.clear();
	hold = 0.0;
}

void DrapeDriverModifier3D::set_tuning(double p_max_stretch, double p_reversed_share, double p_seat_lap_lift, double p_hold_span, double p_hold_time) {
	max_stretch = p_max_stretch;
	reversed_share = p_reversed_share;
	seat_lap_lift = p_seat_lap_lift;
	hold_span = p_hold_span;
	hold_time = p_hold_time;
}

void DrapeDriverModifier3D::set_bend(double p_from, double p_to) {
	bend_from = p_from;
	bend_to = p_to;
}

void DrapeDriverModifier3D::set_legs_layout(const PackedInt32Array &p_bones, const PackedVector3Array &p_ends, const PackedVector2Array &p_radii) {
	ERR_FAIL_COND_MSG(p_ends.size() != p_bones.size() * 2 || p_radii.size() != p_bones.size(), "Two ends and a pair of radii per leg capsule.");
	leg_bones.clear();
	leg_ends.clear();
	leg_radii.clear();
	leg_params.clear();
	for (int k = 0; k < p_bones.size(); k++) {
		leg_bones.push_back(p_bones[k]);
		leg_ends.push_back(p_ends[k * 2]);
		leg_ends.push_back(p_ends[k * 2 + 1]);
		leg_radii.push_back(p_radii[k]);
		leg_params.push_back(StringName(vformat("drape_leg_%d_a", k)));
		leg_params.push_back(StringName(vformat("drape_leg_%d_b", k)));
	}
	capsules.resize(leg_bones.size() * 2);
	capsules.fill(Vector4());
}

void DrapeDriverModifier3D::set_pelvis_bone(int p_bone) {
	pelvis_bone = p_bone;
}

// The hips' centre in the wardrobe mesh's space, and in w the upright of
// the rest pose as the hips carry it, packed as DrapeSeatedTarget.
// pack_normal() packs (octahedral, PELVIS_STEPS a side); w -1 for none.
Vector4 DrapeDriverModifier3D::compute_pelvis(Skeleton3D *p_skeleton, Node *p_wardrobe) const {
	Node3D *wardrobe = Object::cast_to<Node3D>(p_wardrobe);
	ERR_FAIL_NULL_V(p_skeleton, Vector4(0, 0, 0, -1));
	ERR_FAIL_NULL_V(wardrobe, Vector4(0, 0, 0, -1));
	if (pelvis_bone < 0 || pelvis_bone >= p_skeleton->get_bone_count()) {
		return Vector4(0, 0, 0, -1);
	}
	const Transform3D to_mesh = wardrobe->get_global_transform().affine_inverse() * p_skeleton->get_global_transform();
	const Transform3D pose = p_skeleton->get_bone_global_pose(pelvis_bone);
	const Vector3 centre = to_mesh.xform(pose.origin);
	const Basis carried = pose.basis * p_skeleton->get_bone_global_rest(pelvis_bone).basis.inverse();
	Vector3 n = to_mesh.basis.xform(carried.xform(Vector3(0, 1, 0)));
	n /= MAX(Math::abs(n.x) + Math::abs(n.y) + Math::abs(n.z), 1e-12f);
	Vector2 e(n.x, n.y);
	if (n.z < 0.0f) {
		e = Vector2((1.0f - Math::abs(n.y)) * (n.x >= 0.0f ? 1.0f : -1.0f), (1.0f - Math::abs(n.x)) * (n.y >= 0.0f ? 1.0f : -1.0f));
	}
	const Vector2 q = ((e * 0.5f + Vector2(0.5f, 0.5f)) * PELVIS_STEPS).round();
	return Vector4(centre.x, centre.y, centre.z, q.x * (PELVIS_STEPS + 1.0f) + q.y);
}

void DrapeDriverModifier3D::set_rings_node(Node *p_rings) {
	rings_id = p_rings ? p_rings->get_instance_id() : ObjectID();
}

void DrapeDriverModifier3D::set_springs_node(Node *p_springs) {
	springs_id = p_springs ? p_springs->get_instance_id() : ObjectID();
}

void DrapeDriverModifier3D::set_wardrobe_node(Node *p_wardrobe) {
	wardrobe_id = p_wardrobe ? p_wardrobe->get_instance_id() : ObjectID();
}

void DrapeDriverModifier3D::reset_springs_in_pass() {
	reset_springs = true;
}

Quaternion DrapeDriverModifier3D::_turn(const Quaternion &p_swing, float p_weight) {
	if (p_weight == 0.0f) {
		return Quaternion();
	}
	return Quaternion().slerp(p_weight < 0.0f ? p_swing.inverse() : p_swing, Math::abs(p_weight));
}

Quaternion DrapeDriverModifier3D::_swing(Skeleton3D *p_skeleton, int p_k) const {
	const Vector3 from = thigh_rests[p_k].xform(thigh_along[p_k]);
	const Vector3 to = p_skeleton->get_bone_pose_rotation(thighs[p_k]).xform(thigh_along[p_k]);
	if (from.cross(to).length_squared() <= 1e-12f) {
		return Quaternion();
	}
	return Quaternion(from.normalized(), to.normalized());
}

float DrapeDriverModifier3D::_outward_weight(const Quaternion &p_swing, int p_c, float p_weight) const {
	if (p_weight <= 0.0f) {
		return 0.0f;
	}
	const Vector3 down = downs[p_c];
	const Vector3 moved = p_swing.xform(down) - down;
	const float length = moved.length();
	if (length < 1e-6f) {
		return p_weight;
	}
	// Followed whole while the swing carries the column out or along the
	// body, reversed as it turns inward. A swing along the body (a thigh
	// coming forward, under a column at the side) is where the two meet:
	// switched there, a side column turned a whole share and a half the
	// other way in one pass, mid-stride.
	const float inward = Math::smoothstep(0.0f, INWARD_BAND, -moved.dot(outs[p_c]) / length);
	return p_weight * Math::lerp(1.0f, -(float)reversed_share, inward);
}

void DrapeDriverModifier3D::_process_modification(double p_delta) {
	// The physics tick's pass is never drawn, except while the tree is paused,
	// when it is the only one.
	bool paused = is_inside_tree() && get_tree()->is_paused();
	if ((Engine::get_singleton()->is_in_physics_frame() && !paused) || thighs.is_empty()) {
		return;
	}
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton) {
		return;
	}
	const int bone_count = skeleton->get_bone_count();
	for (int k = 0; k < 2; k++) {
		if (thighs[k] < 0 || thighs[k] >= bone_count) {
			return;
		}
	}

	// The hold moves towards what the rings' largest stretch asks for.
	double wanted = 0.0;
	DrapeRingsModifier3D *rings = ObjectDB::get_instance<DrapeRingsModifier3D>(rings_id);
	if (rings && rings->is_active()) {
		const PackedFloat32Array stretch = rings->get_stretch();
		float largest = 0.0f;
		for (int i = 0; i < stretch.size(); i++) {
			largest = MAX(largest, stretch[i]);
		}
		wanted = CLAMP((largest / max_stretch - 1.0) / hold_span, 0.0, 1.0);
	}
	hold = Math::move_toward(hold, wanted, MAX(p_delta, 0.0) / hold_time);

	const Quaternion swing_l = _swing(skeleton, 0);
	const Quaternion swing_r = _swing(skeleton, 1);
	// A thigh lifted far up the hips (bent over, a high stride) runs through the
	// lap's columns as a seated one does, so the lap follows and lifts over it
	// as far as it is lifted, seated or not.
	bent = Vector2();
	if (bend_to > bend_from) {
		for (int k = 0; k < 2; k++) {
			const Vector3 along = skeleton->get_bone_pose_rotation(thighs[k]).xform(thigh_along[k]);
			bent[k] = Math::smoothstep((float)bend_from, (float)bend_to, along.angle_to(-up));
		}
	}
	// How far the legs are into a pose variant's pose, and which: its targets
	// shape the chains only while the springs swing them (§6.4), since the
	// cloth that moves the garment off a seat keeps its own shape.
	pose_variant = -1;
	pose_weight = 0.0;
	if (knees[0] >= 0 && knees[0] < bone_count && knees[1] >= 0 && knees[1] < bone_count) {
		_measure_legs(skeleton);
		SpringBoneSimulator3D *springs = ObjectDB::get_instance<SpringBoneSimulator3D>(springs_id);
		if (pose_variants > 0 && springs && springs->is_active()) {
			_weigh_poses();
		}
	}
	const Quaternion *targets = pose_variant >= 0 ? pose_targets.ptr() + pose_variant * pose_bones.size() : nullptr;
	Vector2 *applied_w = applied.ptrw();
	for (uint32_t c = 0; c < tops.size(); c++) {
		if (tops[c] < 0 || tops[c] >= bone_count) {
			continue;
		}
		Vector2 w = hold > 0.0 ? weights[c].lerp(shares[c], hold) : weights[c];
		// A column pressed onto the seat lies along it: it neither follows the
		// thighs nor lifts over them.
		const float press = c < presses.size() ? presses[c] : 0.0f;
		const float share_sum = shares[c].x + shares[c].y;
		const float lapped = MAX((float)seated, share_sum > 0.0f ? (shares[c].x * bent.x + shares[c].y * bent.y) / share_sum : 0.0f);
		if (lapped > 0.0f) {
			w = w.lerp(shares[c], lapped * laps[c]);
		}
		if (seated > 0.0) {
			w = w.lerp(Vector2(), seated * press);
		}
		const float w_l = _outward_weight(swing_l, c, w.x);
		const float w_r = _outward_weight(swing_r, c, w.y);
		applied_w[c] = Vector2(w_l, w_r);
		Quaternion turned = _turn(swing_l, w_l) * _turn(swing_r, w_r);
		if (seated > 0.0) {
			turned = Quaternion().slerp(seat_turns[c], seated) * turned;
		}
		if (lapped > 0.0f && laps[c] > 0.0f) {
			const Vector3 lift_axis = turned.xform(downs[c]).cross(up);
			if (lift_axis.length_squared() > 1e-8f) {
				turned = Quaternion(lift_axis.normalized(), seat_lap_lift * lapped * laps[c] * (1.0f - press)) * turned;
			}
		}
		Quaternion top = turned * rests[c];
		// A pose's target takes the top as far as the legs are into it, but
		// not a top the seat presses onto it.
		if (targets && c < pose_tops.size() && pose_tops[c] >= 0) {
			top = top.slerp(targets[pose_tops[c]], (float)pose_weight * (1.0f - press * (float)seated));
		}
		skeleton->set_bone_pose_rotation(tops[c], top);
	}
	if (targets) {
		for (uint32_t i = 0; i < pose_bones.size(); i++) {
			const int bone = pose_bones[i];
			if (bone < 0 || bone >= bone_count || pose_is_top[i]) {
				continue;
			}
			skeleton->set_bone_pose_rotation(bone, pose_rests[i].slerp(targets[i], (float)pose_weight));
		}
	}
	// Here, inside the pass, the skeleton holds the tops just set: anywhere
	// else it holds its rests, and the springs would hang from those.
	if (reset_springs) {
		reset_springs = false;
		SpringBoneSimulator3D *springs = ObjectDB::get_instance<SpringBoneSimulator3D>(springs_id);
		if (springs && springs->is_active()) {
			springs->reset();
		}
	}
	_push_legs(skeleton);
	_push_hips(skeleton);
}

PackedVector4Array DrapeDriverModifier3D::compute_leg_capsules(Skeleton3D *p_skeleton, Node *p_wardrobe) {
	Node3D *wardrobe = Object::cast_to<Node3D>(p_wardrobe);
	ERR_FAIL_NULL_V(p_skeleton, capsules);
	ERR_FAIL_NULL_V(wardrobe, capsules);
	const Transform3D to_mesh = wardrobe->get_global_transform().affine_inverse() * p_skeleton->get_global_transform();
	const int bone_count = p_skeleton->get_bone_count();
	Vector4 *out = capsules.ptrw();
	for (uint32_t k = 0; k < leg_bones.size(); k++) {
		const int bone = leg_bones[k];
		// The thighs push seated too: the slab holds the cloth under them, not
		// the lap over them.
		if (bone < 0 || bone >= bone_count) {
			out[k * 2] = Vector4();
			out[k * 2 + 1] = Vector4();
			continue;
		}
		const Transform3D at = to_mesh * p_skeleton->get_bone_global_pose(bone);
		const Vector3 a = at.xform(leg_ends[k * 2]);
		const Vector3 b = at.xform(leg_ends[k * 2 + 1]);
		out[k * 2] = Vector4(a.x, a.y, a.z, leg_radii[k].x);
		out[k * 2 + 1] = Vector4(b.x, b.y, b.z, leg_radii[k].y);
	}
	return capsules;
}

void DrapeDriverModifier3D::_push_legs(Skeleton3D *p_skeleton) {
	GeometryInstance3D *wardrobe = ObjectDB::get_instance<GeometryInstance3D>(wardrobe_id);
	if (!wardrobe || leg_bones.is_empty()) {
		return;
	}
	compute_leg_capsules(p_skeleton, wardrobe);
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID instance = wardrobe->get_instance();
	for (int i = 0; i < capsules.size(); i++) {
		rs->instance_geometry_set_shader_parameter(instance, leg_params[i], capsules[i]);
	}
	rs->instance_geometry_set_shader_parameter(instance, pelvis_param, compute_pelvis(p_skeleton, wardrobe));
}

void DrapeDriverModifier3D::set_seated_hips(int p_bone, const Transform3D &p_bind, Node *p_wardrobe) {
	seated_hips = p_bone;
	seated_bind = p_bind;
	seated_wardrobe_id = p_wardrobe ? p_wardrobe->get_instance_id() : ObjectID();
}

Transform3D DrapeDriverModifier3D::compute_seated_hips(Skeleton3D *p_skeleton, int p_bone, const Transform3D &p_bind, Node *p_wardrobe) {
	Node3D *wardrobe = Object::cast_to<Node3D>(p_wardrobe);
	ERR_FAIL_NULL_V(p_skeleton, Transform3D());
	ERR_FAIL_NULL_V(wardrobe, Transform3D());
	ERR_FAIL_INDEX_V(p_bone, p_skeleton->get_bone_count(), Transform3D());
	// A target vertex is in the mesh's data with the hips at rest: to the
	// skeleton's space, carried from the hips' rest to their pose, then into
	// the mesh's space as the skinned vertex is.
	const Transform3D to_mesh = wardrobe->get_global_transform().affine_inverse() * p_skeleton->get_global_transform();
	return to_mesh * p_skeleton->get_bone_global_pose(p_bone) * p_skeleton->get_bone_global_rest(p_bone).affine_inverse() * p_bind;
}

void DrapeDriverModifier3D::_push_hips(Skeleton3D *p_skeleton) {
	if (seated_hips < 0 || seated_hips >= p_skeleton->get_bone_count()) {
		return;
	}
	GeometryInstance3D *wardrobe = ObjectDB::get_instance<GeometryInstance3D>(seated_wardrobe_id);
	if (!wardrobe) {
		return;
	}
	const Transform3D hips = compute_seated_hips(p_skeleton, seated_hips, seated_bind, wardrobe);
	RenderingServer *rs = RenderingServer::get_singleton();
	const RID instance = wardrobe->get_instance();
	for (int k = 0; k < 3; k++) {
		const Vector3 row = hips.basis.rows[k];
		rs->instance_geometry_set_shader_parameter(instance, hips_params[k], Vector4(row.x, row.y, row.z, hips.origin[k]));
	}
}

void DrapeDriverModifier3D::set_seated(double p_seated) {
	seated = p_seated;
}

double DrapeDriverModifier3D::get_seated() const {
	return seated;
}

double DrapeDriverModifier3D::get_hold() const {
	return hold;
}

PackedVector2Array DrapeDriverModifier3D::get_weights() const {
	return weights;
}

PackedVector2Array DrapeDriverModifier3D::get_applied() const {
	return applied;
}

Vector2 DrapeDriverModifier3D::get_bent() const {
	return bent;
}

PackedFloat32Array DrapeDriverModifier3D::get_legs() const {
	return legs;
}

int DrapeDriverModifier3D::get_pose_variant() const {
	return pose_variant;
}

double DrapeDriverModifier3D::get_pose_weight() const {
	return pose_weight;
}

void DrapeDriverModifier3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_layout", "thighs", "thigh_rests", "thigh_along", "tops", "rests", "shares", "weights", "downs", "outs", "seat_turns", "laps", "up"), &DrapeDriverModifier3D::set_layout);
	ClassDB::bind_method(D_METHOD("clear_layout"), &DrapeDriverModifier3D::clear_layout);
	ClassDB::bind_method(D_METHOD("set_seat_turns", "seat_turns", "presses"), &DrapeDriverModifier3D::set_seat_turns);
	ClassDB::bind_method(D_METHOD("set_tuning", "max_stretch", "reversed_share", "seat_lap_lift", "hold_span", "hold_time"), &DrapeDriverModifier3D::set_tuning);
	ClassDB::bind_method(D_METHOD("set_legs_layout", "bones", "ends", "radii"), &DrapeDriverModifier3D::set_legs_layout);
	ClassDB::bind_method(D_METHOD("set_rings_node", "rings"), &DrapeDriverModifier3D::set_rings_node);
	ClassDB::bind_method(D_METHOD("set_springs_node", "springs"), &DrapeDriverModifier3D::set_springs_node);
	ClassDB::bind_method(D_METHOD("set_wardrobe_node", "wardrobe"), &DrapeDriverModifier3D::set_wardrobe_node);
	ClassDB::bind_method(D_METHOD("reset_springs_in_pass"), &DrapeDriverModifier3D::reset_springs_in_pass);
	ClassDB::bind_method(D_METHOD("compute_leg_capsules", "skeleton", "wardrobe"), &DrapeDriverModifier3D::compute_leg_capsules);
	ClassDB::bind_method(D_METHOD("set_pelvis_bone", "bone"), &DrapeDriverModifier3D::set_pelvis_bone);
	ClassDB::bind_method(D_METHOD("compute_pelvis", "skeleton", "wardrobe"), &DrapeDriverModifier3D::compute_pelvis);
	ClassDB::bind_method(D_METHOD("set_seated_hips", "bone", "bind", "wardrobe"), &DrapeDriverModifier3D::set_seated_hips);
	ClassDB::bind_static_method("DrapeDriverModifier3D", D_METHOD("compute_seated_hips", "skeleton", "bone", "bind", "wardrobe"), &DrapeDriverModifier3D::compute_seated_hips);
	ClassDB::bind_method(D_METHOD("set_seated", "seated"), &DrapeDriverModifier3D::set_seated);
	ClassDB::bind_method(D_METHOD("get_seated"), &DrapeDriverModifier3D::get_seated);
	ClassDB::bind_method(D_METHOD("get_hold"), &DrapeDriverModifier3D::get_hold);
	ClassDB::bind_method(D_METHOD("get_weights"), &DrapeDriverModifier3D::get_weights);
	ClassDB::bind_method(D_METHOD("get_applied"), &DrapeDriverModifier3D::get_applied);
	ClassDB::bind_method(D_METHOD("set_bend", "from", "to"), &DrapeDriverModifier3D::set_bend);
	ClassDB::bind_method(D_METHOD("get_bent"), &DrapeDriverModifier3D::get_bent);
	ClassDB::bind_method(D_METHOD("set_pose_targets", "bones", "rests", "targets", "legs", "knees", "knee_rests", "shin_along", "hips_rest"), &DrapeDriverModifier3D::set_pose_targets);
	ClassDB::bind_method(D_METHOD("clear_pose_targets"), &DrapeDriverModifier3D::clear_pose_targets);
	ClassDB::bind_method(D_METHOD("set_pose_tuning", "from", "stray"), &DrapeDriverModifier3D::set_pose_tuning);
	ClassDB::bind_static_method("DrapeDriverModifier3D", D_METHOD("weigh_pose", "legs", "variant", "from", "stray"), &DrapeDriverModifier3D::weigh_pose);
	ClassDB::bind_static_method("DrapeDriverModifier3D", D_METHOD("side_angle", "v"), &DrapeDriverModifier3D::side_angle);
	ClassDB::bind_method(D_METHOD("get_legs"), &DrapeDriverModifier3D::get_legs);
	ClassDB::bind_method(D_METHOD("get_pose_variant"), &DrapeDriverModifier3D::get_pose_variant);
	ClassDB::bind_method(D_METHOD("get_pose_weight"), &DrapeDriverModifier3D::get_pose_weight);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "seated", PROPERTY_HINT_RANGE, "0,1,0.001", PROPERTY_USAGE_NONE), "set_seated", "get_seated");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hold", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_hold");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "weights", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_weights");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "applied", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_applied");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "legs", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_legs");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pose_variant", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_pose_variant");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pose_weight", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_pose_weight");
}

DrapeDriverModifier3D::DrapeDriverModifier3D() {
	for (int k = 0; k < 3; k++) {
		hips_params[k] = StringName(vformat("drape_hips_%d", k));
	}
}
