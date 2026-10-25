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

void DrapeDriverModifier3D::clear_layout() {
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
	return (p_swing.xform(down) - down).dot(outs[p_c]) < 0.0f ? -p_weight * (float)reversed_share : p_weight;
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
	Vector2 *applied_w = applied.ptrw();
	for (uint32_t c = 0; c < tops.size(); c++) {
		if (tops[c] < 0 || tops[c] >= bone_count) {
			continue;
		}
		Vector2 w = hold > 0.0 ? weights[c].lerp(shares[c], hold) : weights[c];
		if (seated > 0.0) {
			w = w.lerp(shares[c], seated * laps[c]);
		}
		const float w_l = _outward_weight(swing_l, c, w.x);
		const float w_r = _outward_weight(swing_r, c, w.y);
		applied_w[c] = Vector2(w_l, w_r);
		Quaternion turned = _turn(swing_l, w_l) * _turn(swing_r, w_r);
		if (seated > 0.0) {
			turned = Quaternion().slerp(seat_turns[c], seated) * turned;
			const Vector3 lift_axis = turned.xform(downs[c]).cross(up);
			if (laps[c] > 0.0f && lift_axis.length_squared() > 1e-8f) {
				turned = Quaternion(lift_axis.normalized(), seat_lap_lift * seated * laps[c]) * turned;
			}
		}
		skeleton->set_bone_pose_rotation(tops[c], turned * rests[c]);
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
		// The thighs push nothing while seated: the slab holds the lap's cloth.
		if (bone < 0 || bone >= bone_count || (k < 2 && seated > 0.0)) {
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

void DrapeDriverModifier3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_layout", "thighs", "thigh_rests", "thigh_along", "tops", "rests", "shares", "weights", "downs", "outs", "seat_turns", "laps", "up"), &DrapeDriverModifier3D::set_layout);
	ClassDB::bind_method(D_METHOD("clear_layout"), &DrapeDriverModifier3D::clear_layout);
	ClassDB::bind_method(D_METHOD("set_tuning", "max_stretch", "reversed_share", "seat_lap_lift", "hold_span", "hold_time"), &DrapeDriverModifier3D::set_tuning);
	ClassDB::bind_method(D_METHOD("set_legs_layout", "bones", "ends", "radii"), &DrapeDriverModifier3D::set_legs_layout);
	ClassDB::bind_method(D_METHOD("set_rings_node", "rings"), &DrapeDriverModifier3D::set_rings_node);
	ClassDB::bind_method(D_METHOD("set_springs_node", "springs"), &DrapeDriverModifier3D::set_springs_node);
	ClassDB::bind_method(D_METHOD("set_wardrobe_node", "wardrobe"), &DrapeDriverModifier3D::set_wardrobe_node);
	ClassDB::bind_method(D_METHOD("reset_springs_in_pass"), &DrapeDriverModifier3D::reset_springs_in_pass);
	ClassDB::bind_method(D_METHOD("compute_leg_capsules", "skeleton", "wardrobe"), &DrapeDriverModifier3D::compute_leg_capsules);
	ClassDB::bind_method(D_METHOD("set_seated", "seated"), &DrapeDriverModifier3D::set_seated);
	ClassDB::bind_method(D_METHOD("get_seated"), &DrapeDriverModifier3D::get_seated);
	ClassDB::bind_method(D_METHOD("get_hold"), &DrapeDriverModifier3D::get_hold);
	ClassDB::bind_method(D_METHOD("get_weights"), &DrapeDriverModifier3D::get_weights);
	ClassDB::bind_method(D_METHOD("get_applied"), &DrapeDriverModifier3D::get_applied);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "seated", PROPERTY_HINT_RANGE, "0,1,0.001", PROPERTY_USAGE_NONE), "set_seated", "get_seated");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hold", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_hold");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "weights", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_weights");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "applied", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_applied");
}
