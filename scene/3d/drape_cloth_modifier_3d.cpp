/**************************************************************************/
/*  drape_cloth_modifier_3d.cpp                                           */
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

#include "drape_cloth_modifier_3d.h"

#include "core/config/engine.h"
#include "core/math/geometry_3d.h"
#include "core/object/class_db.h"
#include "scene/main/scene_tree.h"

void DrapeClothModifier3D::set_layout(int p_center_bone, const PackedInt32Array &p_bones, const PackedInt32Array &p_parents, const PackedFloat32Array &p_tip_lengths, const PackedInt32Array &p_links) {
	clear_layout();
	const int n = p_bones.size();
	ERR_FAIL_COND_MSG(p_parents.size() != n || p_tip_lengths.size() != n, "One parent and one tip length per particle.");
	ERR_FAIL_COND_MSG(p_links.size() % 3 != 0, "Three ints per link: its two particles and its kind.");
	for (int p = 0; p < n; p++) {
		// Each column is listed top to tip: a particle's parent is the one
		// before it, or none for a column's top.
		ERR_FAIL_COND_MSG(p_parents[p] != -1 && p_parents[p] != p - 1, "A particle's parent must be the one listed before it, or -1 for a column's top.");
		ERR_FAIL_COND_MSG(p_bones[p] < 0 && p_parents[p] < 0, "A column's top must be a bone.");
		ERR_FAIL_COND_MSG(p_bones[p] < 0 && p + 1 < n && p_parents[p + 1] == p, "Only a column's last particle may be a tip.");
	}
	for (int k = 0; k < p_links.size(); k += 3) {
		ERR_FAIL_COND_MSG(p_links[k] < 0 || p_links[k] >= n || p_links[k + 1] < 0 || p_links[k + 1] >= n, "A link names a particle that is not there.");
		ERR_FAIL_COND_MSG(p_links[k + 2] < LINK_COLUMN || p_links[k + 2] > LINK_DIAGONAL, "A link's kind is not a LinkKind.");
	}

	center_bone = p_center_bone;
	deepest = 1;
	for (int p = 0; p < n; p++) {
		bones.push_back(p_bones[p]);
		parents.push_back(p_parents[p]);
		tip_lengths.push_back(p_tip_lengths[p]);
		const int ring = p_parents[p] < 0 ? 0 : ring_of[p - 1] + 1;
		ring_of.push_back(ring);
		deepest = MAX(deepest, ring);
		if (p_parents[p] < 0) {
			column_start.push_back(p);
			column_size.push_back(0);
		}
		column_size[column_size.size() - 1]++;
	}
	for (int k = 0; k < p_links.size(); k += 3) {
		Link link;
		link.a = p_links[k];
		link.b = p_links[k + 1];
		link.kind = p_links[k + 2];
		links.push_back(link);
	}
	x.resize(n);
	previous.resize(n);
	hang.resize(n);
	mass_inverse.resize(n);
	input_rotation.resize(n);
	input_origin.resize(n);
	root_parent_rotation.resize(column_start.size());
	for (int p = 0; p < n; p++) {
		mass_inverse[p] = parents[p] < 0 ? 0.0f : 1.0f;
	}
}

void DrapeClothModifier3D::set_colliders(const PackedInt32Array &p_bones, const PackedVector3Array &p_ends, const PackedVector2Array &p_radii) {
	capsules.clear();
	ERR_FAIL_COND_MSG(p_ends.size() != p_bones.size() * 2 || p_radii.size() != p_bones.size(), "Two ends and one pair of radii per capsule.");
	for (int k = 0; k < p_bones.size(); k++) {
		Capsule capsule;
		capsule.bone = p_bones[k];
		capsule.a = p_ends[k * 2];
		capsule.b = p_ends[k * 2 + 1];
		capsule.radius_a = p_radii[k].x;
		capsule.radius_b = p_radii[k].y;
		capsules.push_back(capsule);
	}
	placed.resize(capsules.size());
	// The allowances are measured against these.
	measured = false;
}

void DrapeClothModifier3D::clear_layout() {
	center_bone = -1;
	bones.clear();
	parents.clear();
	tip_lengths.clear();
	ring_of.clear();
	column_start.clear();
	column_size.clear();
	links.clear();
	measured = false;
	started = false;
}

void DrapeClothModifier3D::reset() {
	started = false;
}

void DrapeClothModifier3D::_process_modification(double p_delta) {
	// The physics tick's pass is never drawn, except while the tree is paused,
	// when it is the only one.
	bool paused = is_inside_tree() && get_tree()->is_paused();
	if ((Engine::get_singleton()->is_in_physics_frame() && !paused) || center_bone < 0 || bones.is_empty()) {
		return;
	}
	Skeleton3D *skeleton = get_skeleton();
	if (!skeleton || center_bone >= skeleton->get_bone_count()) {
		return;
	}
	const Transform3D center = skeleton->get_bone_global_pose(center_bone);
	if (!_read(skeleton, center)) {
		return;
	}
	if (p_delta > 0.0) {
		_step(center, p_delta);
	}
	_write(skeleton, center);
}

bool DrapeClothModifier3D::_read(Skeleton3D *p_skeleton, const Transform3D &p_center) {
	const int bone_count = p_skeleton->get_bone_count();
	for (uint32_t p = 0; p < bones.size(); p++) {
		if (bones[p] >= bone_count) {
			return false;
		}
	}
	for (uint32_t k = 0; k < capsules.size(); k++) {
		if (capsules[k].bone < 0 || capsules[k].bone >= bone_count) {
			return false;
		}
	}
	if (!measured) {
		// Link lengths from the rest: the cloth as modelled.
		LocalVector<Vector3> rest;
		rest.resize(bones.size());
		for (uint32_t p = 0; p < bones.size(); p++) {
			if (bones[p] >= 0) {
				rest[p] = p_skeleton->get_bone_global_rest(bones[p]).origin;
			} else {
				const Transform3D above = p_skeleton->get_bone_global_rest(bones[p - 1]);
				rest[p] = above.origin + above.basis.get_column(1).normalized() * tip_lengths[p];
			}
		}
		for (uint32_t k = 0; k < links.size(); k++) {
			links[k].rest = rest[links[k].a].distance_to(rest[links[k].b]);
		}
		// And how deep each sits in each capsule there, with the thickness
		// the pass collides at.
		const uint32_t m = capsules.size();
		particle_allowance.resize(bones.size() * m);
		link_allowance.resize(links.size() * m);
		for (uint32_t k = 0; k < m; k++) {
			const Transform3D pose = p_skeleton->get_bone_global_rest(capsules[k].bone);
			Capsule at_rest = capsules[k];
			at_rest.a = pose.xform(capsules[k].a);
			at_rest.b = pose.xform(capsules[k].b);
			at_rest.radius_a += thickness;
			at_rest.radius_b += thickness;
			for (uint32_t p = 0; p < bones.size(); p++) {
				particle_allowance[p * m + k] = _depth(rest[p], rest[p], at_rest);
			}
			for (uint32_t l = 0; l < links.size(); l++) {
				link_allowance[l * m + k] = _depth(rest[links[l].a], rest[links[l].b], at_rest);
			}
		}
		measured = true;
	}
	const Transform3D to_center = p_center.affine_inverse();
	float jump = 0.0f;
	for (uint32_t p = 0; p < bones.size(); p++) {
		Vector3 at;
		if (bones[p] >= 0) {
			const Transform3D pose = p_skeleton->get_bone_global_pose(bones[p]);
			input_rotation[p] = pose.basis.get_rotation_quaternion();
			input_origin[p] = pose.origin;
			at = pose.origin;
		} else {
			at = input_origin[p - 1] + input_rotation[p - 1].xform(Vector3(0, tip_lengths[p], 0));
		}
		const Vector3 now = to_center.xform(at);
		// Only the first joints below the pinned tops are watched: they move
		// with the thighs the driver follows, where a joint further down may
		// swing that far in one pass of a stride.
		if (started && ring_of[p] == 1) {
			jump = MAX(jump, now.distance_to(hang[p]));
		}
		hang[p] = now;
	}
	for (uint32_t c = 0; c < column_start.size(); c++) {
		const int up = p_skeleton->get_bone_parent(bones[column_start[c]]);
		root_parent_rotation[c] = up >= 0 ? p_skeleton->get_bone_global_pose(up).basis.get_rotation_quaternion() : Quaternion();
	}
	// A pose that jumps (a restore, a cut in the animation) starts the cloth
	// over from it, rather than dragging what hung before through the body.
	if (started && jump > snap_distance) {
		started = false;
		resets++;
	}
	if (!started) {
		for (uint32_t p = 0; p < bones.size(); p++) {
			x[p] = hang[p];
			previous[p] = hang[p];
		}
		started = true;
	}
	for (uint32_t k = 0; k < capsules.size(); k++) {
		const Transform3D pose = to_center * p_skeleton->get_bone_global_pose(capsules[k].bone);
		placed[k].a = pose.xform(capsules[k].a);
		placed[k].b = pose.xform(capsules[k].b);
		placed[k].radius_a = capsules[k].radius_a + thickness;
		placed[k].radius_b = capsules[k].radius_b + thickness;
	}
	return true;
}

void DrapeClothModifier3D::_step(const Transform3D &p_center, double p_delta) {
	const uint32_t n = bones.size();
	const Vector3 down = p_center.basis.inverse().xform(Vector3(0, -1, 0)).normalized();
	const Vector3 fall = down * (float)(gravity * p_delta * p_delta);
	const float keep = 1.0f - (float)damping;
	for (uint32_t p = 0; p < n; p++) {
		if (mass_inverse[p] == 0.0f) {
			x[p] = hang[p];
			previous[p] = hang[p];
			continue;
		}
		const Vector3 velocity = (x[p] - previous[p]) * keep;
		previous[p] = x[p];
		x[p] += velocity + fall;
	}
	// Each joint toward its hang from the one above, top down: firmly near
	// the top, where the pose's follow of the thighs must hold, and loosely
	// toward the hem.
	for (uint32_t c = 0; c < column_start.size(); c++) {
		const int start = column_start[c];
		for (int k = 1; k < column_size[c]; k++) {
			const int p = start + k;
			const float share = deepest > 1 ? float(ring_of[p] - 1) / float(deepest - 1) : 0.0f;
			const float pull = Math::lerp((float)top_pull, (float)hem_pull, CLAMP(share, 0.0f, 1.0f));
			const Vector3 target = x[p - 1] + (hang[p] - hang[p - 1]);
			x[p] = x[p].lerp(target, pull);
		}
	}
	const float stiffness[3] = { 1.0f, (float)ring_stiffness, (float)shear_stiffness };
	const uint32_t m = placed.size();
	for (int it = 0; it < iterations; it++) {
		for (uint32_t k = 0; k < links.size(); k++) {
			const Link &link = links[k];
			const Vector3 d = x[link.b] - x[link.a];
			const float length = d.length();
			if (length < 1e-7f) {
				continue;
			}
			// Round a ring and across it, cloth gathers freely; it only
			// resists being stretched.
			if (link.kind != LINK_COLUMN && length <= link.rest) {
				continue;
			}
			const float w = mass_inverse[link.a] + mass_inverse[link.b];
			if (w == 0.0f) {
				continue;
			}
			const Vector3 fix = d * ((length - link.rest) / length / w * stiffness[link.kind]);
			x[link.a] += fix * mass_inverse[link.a];
			x[link.b] -= fix * mass_inverse[link.b];
		}
		for (uint32_t k = 0; k < placed.size(); k++) {
			const Capsule &capsule = placed[k];
			// Most of the cloth is nowhere near a given leg: a box round the
			// capsule rules out a particle or a link with both ends outside
			// it on one side, before the exact test.
			const float reach = MAX(capsule.radius_a, capsule.radius_b);
			const Vector3 low = capsule.a.min(capsule.b) - Vector3(reach, reach, reach);
			const Vector3 high = capsule.a.max(capsule.b) + Vector3(reach, reach, reach);
			for (uint32_t p = 0; p < n; p++) {
				const Vector3 &q = x[p];
				if (mass_inverse[p] > 0.0f && q.x >= low.x && q.y >= low.y && q.z >= low.z && q.x <= high.x && q.y <= high.y && q.z <= high.z) {
					_collide(p, capsule, particle_allowance[p * m + k]);
				}
			}
			for (uint32_t l = 0; l < links.size(); l++) {
				const Vector3 &q0 = x[links[l].a];
				const Vector3 &q1 = x[links[l].b];
				if ((q0.x < low.x && q1.x < low.x) || (q0.x > high.x && q1.x > high.x) ||
						(q0.y < low.y && q1.y < low.y) || (q0.y > high.y && q1.y > high.y) ||
						(q0.z < low.z && q1.z < low.z) || (q0.z > high.z && q1.z > high.z)) {
					continue;
				}
				_collide_link(links[l].a, links[l].b, capsule, link_allowance[l * m + k]);
			}
		}
	}
	// Down each column once more, top down, so the bones keep their lengths.
	for (uint32_t c = 0; c < column_start.size(); c++) {
		const int start = column_start[c];
		for (int k = 1; k < column_size[c]; k++) {
			const int p = start + k;
			const Vector3 d = x[p] - x[p - 1];
			if (d.length_squared() > 1e-14f) {
				x[p] = x[p - 1] + d.normalized() * hang[p].distance_to(hang[p - 1]);
			}
		}
	}
}

// How far inside `p_capsule` the segment from `p_a` to `p_b` reaches at its
// nearest (a point when the two are one): 0 outside it.
float DrapeClothModifier3D::_depth(const Vector3 &p_a, const Vector3 &p_b, const Capsule &p_capsule) {
	Vector3 on_segment;
	Vector3 on_axis;
	Geometry3D::get_closest_points_between_segments(p_a, p_b, p_capsule.a, p_capsule.b, on_segment, on_axis);
	const Vector3 axis = p_capsule.b - p_capsule.a;
	const float t = CLAMP((on_axis - p_capsule.a).dot(axis) / MAX(axis.dot(axis), 1e-8f), 0.0f, 1.0f);
	const float radius = Math::lerp(p_capsule.radius_a, p_capsule.radius_b, t);
	return MAX(radius - on_segment.distance_to(on_axis), 0.0f);
}

void DrapeClothModifier3D::_collide(int p_particle, const Capsule &p_capsule, float p_allowance) {
	const Vector3 &a = p_capsule.a;
	const Vector3 axis = p_capsule.b - a;
	const Vector3 p = x[p_particle];
	const float t = CLAMP((p - a).dot(axis) / MAX(axis.dot(axis), 1e-8f), 0.0f, 1.0f);
	const float radius = Math::lerp(p_capsule.radius_a, p_capsule.radius_b, t) - p_allowance;
	const Vector3 nearest = a + axis * t;
	const float d = p.distance_to(nearest);
	if (d >= radius || d < 1e-6f) {
		return;
	}
	x[p_particle] = nearest + (p - nearest) * (radius / d);
}

void DrapeClothModifier3D::_collide_link(int p_a, int p_b, const Capsule &p_capsule, float p_allowance) {
	const float w_a = mass_inverse[p_a];
	const float w_b = mass_inverse[p_b];
	if (w_a == 0.0f && w_b == 0.0f) {
		return;
	}
	Vector3 on_link;
	Vector3 on_axis;
	Geometry3D::get_closest_points_between_segments(x[p_a], x[p_b], p_capsule.a, p_capsule.b, on_link, on_axis);
	const Vector3 axis = p_capsule.b - p_capsule.a;
	const float t_axis = CLAMP((on_axis - p_capsule.a).dot(axis) / MAX(axis.dot(axis), 1e-8f), 0.0f, 1.0f);
	const float radius = Math::lerp(p_capsule.radius_a, p_capsule.radius_b, t_axis) - p_allowance;
	const Vector3 apart = on_link - on_axis;
	const float d = apart.length();
	if (d >= radius || d < 1e-6f) {
		return;
	}
	const Vector3 normal = apart / d;
	const Vector3 link = x[p_b] - x[p_a];
	const float t = CLAMP((on_link - x[p_a]).dot(link) / MAX(link.dot(link), 1e-8f), 0.0f, 1.0f);
	// Each end moved by its share of the contact, weighted by where along the
	// link it is and whether it may move at all.
	const float s_a = (1.0f - t) * w_a;
	const float s_b = t * w_b;
	const float sum = s_a * s_a + s_b * s_b;
	if (sum < 1e-8f) {
		return;
	}
	const float scale = (radius - d) / sum;
	x[p_a] += normal * (scale * s_a);
	x[p_b] += normal * (scale * s_b);
}

void DrapeClothModifier3D::_write(Skeleton3D *p_skeleton, const Transform3D &p_center) {
	// Each column's bones turned top down to point at the particle below,
	// each carried by the turns above it.
	for (uint32_t c = 0; c < column_start.size(); c++) {
		const int start = column_start[c];
		const int size = column_size[c];
		Quaternion parent = root_parent_rotation[c];
		Quaternion carry;
		Vector3 origin = input_origin[start];
		for (int k = 0; k < size - 1; k++) {
			const int i = start + k;
			const int child = i + 1;
			const Quaternion carried = carry * input_rotation[i];
			const Vector3 local = bones[child] >= 0
					? input_rotation[i].xform_inv(input_origin[child] - input_origin[i])
					: Vector3(0, tip_lengths[child], 0);
			const Vector3 now = carried.xform(local);
			const Vector3 to = p_center.xform(x[child]) - origin;
			Quaternion global = carried;
			if (now.length_squared() > 1e-10f && to.length_squared() > 1e-10f) {
				global = (Quaternion(now.normalized(), to.normalized()) * carried).normalized();
			}
			p_skeleton->set_bone_pose_rotation(bones[i], (parent.inverse() * global).normalized());
			carry = global * input_rotation[i].inverse();
			origin += global.xform(local);
			parent = global;
		}
	}
}

void DrapeClothModifier3D::set_iterations(int p_iterations) {
	iterations = MAX(p_iterations, 1);
}

int DrapeClothModifier3D::get_iterations() const {
	return iterations;
}

void DrapeClothModifier3D::set_top_pull(double p_pull) {
	top_pull = CLAMP(p_pull, 0.0, 1.0);
}

double DrapeClothModifier3D::get_top_pull() const {
	return top_pull;
}

void DrapeClothModifier3D::set_hem_pull(double p_pull) {
	hem_pull = CLAMP(p_pull, 0.0, 1.0);
}

double DrapeClothModifier3D::get_hem_pull() const {
	return hem_pull;
}

void DrapeClothModifier3D::set_damping(double p_damping) {
	damping = CLAMP(p_damping, 0.0, 1.0);
}

double DrapeClothModifier3D::get_damping() const {
	return damping;
}

void DrapeClothModifier3D::set_gravity(double p_gravity) {
	gravity = p_gravity;
}

double DrapeClothModifier3D::get_gravity() const {
	return gravity;
}

void DrapeClothModifier3D::set_ring_stiffness(double p_stiffness) {
	ring_stiffness = CLAMP(p_stiffness, 0.0, 1.0);
}

double DrapeClothModifier3D::get_ring_stiffness() const {
	return ring_stiffness;
}

void DrapeClothModifier3D::set_shear_stiffness(double p_stiffness) {
	shear_stiffness = CLAMP(p_stiffness, 0.0, 1.0);
}

double DrapeClothModifier3D::get_shear_stiffness() const {
	return shear_stiffness;
}

void DrapeClothModifier3D::set_thickness(double p_thickness) {
	thickness = MAX(p_thickness, 0.0);
}

double DrapeClothModifier3D::get_thickness() const {
	return thickness;
}

void DrapeClothModifier3D::set_snap_distance(double p_distance) {
	snap_distance = MAX(p_distance, 0.0);
}

double DrapeClothModifier3D::get_snap_distance() const {
	return snap_distance;
}

int DrapeClothModifier3D::get_resets() const {
	return resets;
}

PackedVector3Array DrapeClothModifier3D::get_particles() const {
	PackedVector3Array out;
	out.resize(x.size());
	for (uint32_t p = 0; p < x.size(); p++) {
		out.set(p, x[p]);
	}
	return out;
}

void DrapeClothModifier3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_layout", "center_bone", "bones", "parents", "tip_lengths", "links"), &DrapeClothModifier3D::set_layout);
	ClassDB::bind_method(D_METHOD("set_colliders", "bones", "ends", "radii"), &DrapeClothModifier3D::set_colliders);
	ClassDB::bind_method(D_METHOD("clear_layout"), &DrapeClothModifier3D::clear_layout);
	ClassDB::bind_method(D_METHOD("reset"), &DrapeClothModifier3D::reset);
	ClassDB::bind_method(D_METHOD("set_iterations", "iterations"), &DrapeClothModifier3D::set_iterations);
	ClassDB::bind_method(D_METHOD("get_iterations"), &DrapeClothModifier3D::get_iterations);
	ClassDB::bind_method(D_METHOD("set_top_pull", "pull"), &DrapeClothModifier3D::set_top_pull);
	ClassDB::bind_method(D_METHOD("get_top_pull"), &DrapeClothModifier3D::get_top_pull);
	ClassDB::bind_method(D_METHOD("set_hem_pull", "pull"), &DrapeClothModifier3D::set_hem_pull);
	ClassDB::bind_method(D_METHOD("get_hem_pull"), &DrapeClothModifier3D::get_hem_pull);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &DrapeClothModifier3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &DrapeClothModifier3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &DrapeClothModifier3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &DrapeClothModifier3D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_ring_stiffness", "stiffness"), &DrapeClothModifier3D::set_ring_stiffness);
	ClassDB::bind_method(D_METHOD("get_ring_stiffness"), &DrapeClothModifier3D::get_ring_stiffness);
	ClassDB::bind_method(D_METHOD("set_shear_stiffness", "stiffness"), &DrapeClothModifier3D::set_shear_stiffness);
	ClassDB::bind_method(D_METHOD("get_shear_stiffness"), &DrapeClothModifier3D::get_shear_stiffness);
	ClassDB::bind_method(D_METHOD("set_thickness", "thickness"), &DrapeClothModifier3D::set_thickness);
	ClassDB::bind_method(D_METHOD("get_thickness"), &DrapeClothModifier3D::get_thickness);
	ClassDB::bind_method(D_METHOD("set_snap_distance", "distance"), &DrapeClothModifier3D::set_snap_distance);
	ClassDB::bind_method(D_METHOD("get_snap_distance"), &DrapeClothModifier3D::get_snap_distance);
	ClassDB::bind_method(D_METHOD("get_resets"), &DrapeClothModifier3D::get_resets);
	ClassDB::bind_method(D_METHOD("get_particles"), &DrapeClothModifier3D::get_particles);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "iterations", PROPERTY_HINT_RANGE, "1,32,1"), "set_iterations", "get_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "top_pull", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_top_pull", "get_top_pull");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "hem_pull", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_hem_pull", "get_hem_pull");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity", PROPERTY_HINT_RANGE, "0,20,0.01,or_greater,suffix:m/s²"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ring_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_ring_stiffness", "get_ring_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shear_stiffness", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_shear_stiffness", "get_shear_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "thickness", PROPERTY_HINT_RANGE, "0,0.1,0.001,or_greater,suffix:m"), "set_thickness", "get_thickness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "snap_distance", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater,suffix:m"), "set_snap_distance", "get_snap_distance");

	BIND_ENUM_CONSTANT(LINK_COLUMN);
	BIND_ENUM_CONSTANT(LINK_RING);
	BIND_ENUM_CONSTANT(LINK_DIAGONAL);
}
