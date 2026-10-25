/**************************************************************************/
/*  drape_rings_modifier_3d.cpp                                           */
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

#include "drape_rings_modifier_3d.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "scene/main/scene_tree.h"

void DrapeRingsModifier3D::set_layout(int p_center_bone, const PackedVector3Array &p_axes, const PackedInt32Array &p_bones, const PackedInt32Array &p_column_start, const PackedInt32Array &p_column_size, const PackedInt32Array &p_rings, const PackedFloat32Array &p_ring_rest, const PackedInt32Array &p_members, const PackedFloat32Array &p_rest_angles) {
	clear_layout();
	ERR_FAIL_COND_MSG(p_axes.size() % 3 != 0, "Three axis vectors per panel: origin, down, e1.");
	ERR_FAIL_COND_MSG(p_column_start.size() != p_column_size.size(), "One start and one size per column.");
	ERR_FAIL_COND_MSG(p_rings.size() != p_ring_rest.size() * 5, "Five ints per ring (panel, index, start, size, closed) and one rest length.");
	ERR_FAIL_COND_MSG(p_members.size() != p_rest_angles.size(), "One rest angle per ring member.");
	for (int i = 0; i < p_column_start.size(); i++) {
		ERR_FAIL_COND_MSG(p_column_start[i] < 0 || p_column_size[i] < 0 || p_column_start[i] + p_column_size[i] > p_bones.size(), "A column runs past the bones.");
	}
	for (int k = 0; k < p_ring_rest.size(); k++) {
		const int *r = p_rings.ptr() + k * 5;
		ERR_FAIL_COND_MSG(r[0] < 0 || r[0] >= p_axes.size() / 3, "A ring names a panel that is not there.");
		ERR_FAIL_COND_MSG(r[1] < 1 || r[3] < 2 || r[2] < 0 || r[2] + r[3] > p_members.size(), "A ring's members run past the list, or it has fewer than two, or it is the line.");
		for (int m = 0; m < r[3]; m++) {
			int c = p_members[r[2] + m];
			ERR_FAIL_COND_MSG(c < 0 || c >= p_column_start.size() || r[1] >= p_column_size[c], "A ring names a column, or a joint down it, that is not there.");
		}
	}

	center_bone = p_center_bone;
	for (int i = 0; i < p_axes.size(); i++) {
		axes.push_back(p_axes[i]);
	}
	for (int i = 0; i < p_bones.size(); i++) {
		bones.push_back(p_bones[i]);
	}
	for (int i = 0; i < p_column_start.size(); i++) {
		column_start.push_back(p_column_start[i]);
		column_size.push_back(p_column_size[i]);
	}
	int widest = 0;
	for (int k = 0; k < p_ring_rest.size(); k++) {
		const int *r = p_rings.ptr() + k * 5;
		Ring ring;
		ring.panel = r[0];
		ring.index = r[1];
		ring.start = r[2];
		ring.size = r[3];
		ring.closed = r[4] != 0;
		ring.rest = p_ring_rest[k];
		rings.push_back(ring);
		widest = MAX(widest, ring.size);
	}
	for (int i = 0; i < p_members.size(); i++) {
		members.push_back(p_members[i]);
		rest_angles.push_back(p_rest_angles[i]);
	}

	origin.resize(bones.size());
	rotation.resize(bones.size());
	after.resize(bones.size());
	turned.resize(bones.size());
	motion_rotation.resize(column_start.size());
	motion_origin.resize(column_start.size());
	at.resize(widest);
	offset.resize(widest);
	stretch.resize(rings.size());
	held.resize(rings.size());
	stretch.fill(0.0);
	held.fill(0);
}

void DrapeRingsModifier3D::clear_layout() {
	center_bone = -1;
	axes.clear();
	bones.clear();
	column_start.clear();
	column_size.clear();
	rings.clear();
	members.clear();
	rest_angles.clear();
	stretch.clear();
	held.clear();
	largest_turn = 0.0;
}

void DrapeRingsModifier3D::_process_modification(double p_delta) {
	// The physics tick's pass is never drawn, except while the tree is paused,
	// when it is the only one.
	bool paused = is_inside_tree() && get_tree()->is_paused();
	if ((Engine::get_singleton()->is_in_physics_frame() && !paused) || center_bone < 0 || rings.is_empty()) {
		return;
	}
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton || center_bone >= skeleton->get_bone_count()) {
		return;
	}
	_even(skeleton);
}

void DrapeRingsModifier3D::_even(Skeleton3D *p_skeleton) {
	const int bone_count = p_skeleton->get_bone_count();
	for (uint32_t i = 0; i < bones.size(); i++) {
		if (bones[i] < 0 || bones[i] >= bone_count) {
			return;
		}
	}
	const Transform3D center = p_skeleton->get_bone_global_pose(center_bone);
	const Quaternion center_rotation = center.basis.get_rotation_quaternion();
	largest_turn = 0.0;
	for (uint32_t i = 0; i < bones.size(); i++) {
		const Transform3D pose = p_skeleton->get_bone_global_pose(bones[i]);
		origin[i] = pose.origin;
		rotation[i] = pose.basis.get_rotation_quaternion();
		after[i] = Quaternion();
		turned[i] = 0;
	}
	for (uint32_t c = 0; c < column_start.size(); c++) {
		motion_rotation[c] = Quaternion();
		motion_origin[c] = Vector3();
	}

	int panel = -1;
	Vector3 axis_origin;
	Vector3 down;
	Vector3 e1;
	Vector3 e2;
	float *stretch_w = stretch.ptrw();
	uint8_t *held_w = held.ptrw();
	for (uint32_t k = 0; k < rings.size(); k++) {
		const Ring &ring = rings[k];
		if (ring.panel != panel) {
			panel = ring.panel;
			axis_origin = center.xform(axes[panel * 3]);
			down = center.basis.xform(axes[panel * 3 + 1]).normalized();
			e1 = center.basis.xform(axes[panel * 3 + 2]).normalized();
			e2 = down.cross(e1);
		}
		const int r = ring.index;
		const int start = ring.start;
		const int size = ring.size;
		double sum_sin = 0.0;
		double sum_cos = 0.0;
		for (int m = 0; m < size; m++) {
			const int c = members[start + m];
			const Vector3 p = motion_rotation[c].xform(origin[column_start[c] + r]) + motion_origin[c];
			at[m] = p;
			const Vector3 q = p - axis_origin;
			const float o = Math::wrapf(Math::atan2(q.dot(e2), q.dot(e1)) - rest_angles[start + m], (float)-Math::PI, (float)Math::PI);
			offset[m] = o;
			sum_sin += Math::sin(o);
			sum_cos += Math::cos(o);
		}
		const float mean = Math::atan2(sum_sin, sum_cos);
		// An open panel's edges swing free; between them the turn runs evenly
		// by rest angle.
		const float span = Math::wrapf(rest_angles[start + size - 1] - rest_angles[start], (float)-Math::PI, (float)Math::PI);
		double length = 0.0;
		Vector3 first;
		Vector3 last;
		for (int m = 0; m < size; m++) {
			const int c = members[start + m];
			float target = mean;
			if (!ring.closed) {
				const float f = Math::abs(span) < 1e-5f ? 0.0f : Math::wrapf(rest_angles[start + m] - rest_angles[start], (float)-Math::PI, (float)Math::PI) / span;
				target = Math::lerp(offset[0], offset[size - 1], f);
			}
			const float turn = Math::wrapf(target - offset[m], (float)-Math::PI, (float)Math::PI);
			const Vector3 p = at[m];
			const Vector3 goal = axis_origin + (p - axis_origin).rotated(down, turn);
			const int i = column_start[c] + r;
			const Vector3 pivot = motion_rotation[c].xform(origin[i - 1]) + motion_origin[c];
			const Vector3 from = p - pivot;
			const Vector3 to = goal - pivot;
			Vector3 placed = p;
			if (from.length_squared() > 1e-10f && to.length_squared() > 1e-10f) {
				const Quaternion turned_by(from.normalized(), to.normalized());
				largest_turn = MAX(largest_turn, (double)turned_by.get_angle());
				// The step x -> turned_by * (x - pivot) + pivot, after the
				// column's motion so far.
				motion_rotation[c] = turned_by * motion_rotation[c];
				motion_origin[c] = turned_by.xform(motion_origin[c] - pivot) + pivot;
				placed = turned_by.xform(from) + pivot;
			}
			// The bone above the joint carries the motion after its ring.
			after[i - 1] = motion_rotation[c];
			turned[i - 1] = 1;
			if (m == 0) {
				first = placed;
			} else {
				length += last.distance_to(placed);
			}
			last = placed;
		}
		if (ring.closed && size > 2) {
			length += last.distance_to(first);
		}
		stretch_w[k] = length / MAX((double)ring.rest, 1e-6);
		held_w[k] = stretch_w[k] > max_stretch ? 1 : 0;
	}

	// Each bone turned by the ring below it, and carried by every ring above:
	// its pose is its column's motion after the next ring's turn. A ring not
	// turned leaves the motion as the ring above left it, and the hem joint
	// moves as the bone above it.
	for (uint32_t c = 0; c < column_start.size(); c++) {
		const int start = column_start[c];
		const int size = column_size[c];
		Quaternion parent = center_rotation;
		Quaternion motion;
		for (int j = 0; j < size; j++) {
			const int i = start + j;
			if (j < size - 1 && turned[i]) {
				motion = after[i];
			}
			const Quaternion global = motion * rotation[i];
			p_skeleton->set_bone_pose_rotation(bones[i], parent.inverse() * global);
			parent = global;
		}
	}
}

void DrapeRingsModifier3D::set_max_stretch(double p_max_stretch) {
	max_stretch = p_max_stretch;
}

double DrapeRingsModifier3D::get_max_stretch() const {
	return max_stretch;
}

PackedFloat32Array DrapeRingsModifier3D::get_stretch() const {
	return stretch;
}

PackedByteArray DrapeRingsModifier3D::get_held() const {
	return held;
}

double DrapeRingsModifier3D::get_largest_turn() const {
	return largest_turn;
}

void DrapeRingsModifier3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_layout", "center_bone", "axes", "bones", "column_start", "column_size", "rings", "ring_rest", "members", "rest_angles"), &DrapeRingsModifier3D::set_layout);
	ClassDB::bind_method(D_METHOD("clear_layout"), &DrapeRingsModifier3D::clear_layout);
	ClassDB::bind_method(D_METHOD("set_max_stretch", "max_stretch"), &DrapeRingsModifier3D::set_max_stretch);
	ClassDB::bind_method(D_METHOD("get_max_stretch"), &DrapeRingsModifier3D::get_max_stretch);
	ClassDB::bind_method(D_METHOD("get_stretch"), &DrapeRingsModifier3D::get_stretch);
	ClassDB::bind_method(D_METHOD("get_held"), &DrapeRingsModifier3D::get_held);
	ClassDB::bind_method(D_METHOD("get_largest_turn"), &DrapeRingsModifier3D::get_largest_turn);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_stretch", PROPERTY_HINT_RANGE, "1,2,0.001,or_greater"), "set_max_stretch", "get_max_stretch");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "stretch", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_stretch");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_BYTE_ARRAY, "held", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_held");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "largest_turn", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "", "get_largest_turn");
}
