/**************************************************************************/
/*  body_contacts_solver.h                                                */
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

#pragma once

#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "scene/3d/body_profile.h"

class Skeleton3D;

// Moves a played clip's hand onto the same place on this body, for
// AdventureAI's body contacts (docs/BODY_CONTACTS.md there): a body's
// contact points skinned to the animated skeleton's pose, each hand's target
// and the torso's lean toward it, and the hand's turn that keeps the forearm
// out of the torso. The weights, the scheduling, the arm requests and the
// tuning are a script's (BodyContacts); the arithmetic is here, since a
// script's took 90 us a character a tick.
class BodyContactsSolver : public RefCounted {
	GDCLASS(BodyContactsSolver, RefCounted);

	// A base bone read off the animated skeleton into the skinned one's space.
	struct Bone {
		int index = -1;
		Basis rest_turn;
		Transform3D skinned_rest_inverse;
	};
	struct Arm {
		int shoulder = -1;
		int elbow = -1;
		int hand = -1;
		real_t upper = 0.0;
		real_t fore = 0.0;
		real_t forearm_radius = -1.0;
	};
	struct Hand {
		Vector3 clip_point;
		Transform3D local_target;
		Vector3 point_in_hand;
		bool reachable = true;
		Vector3 lean_axis;
		int anchor_bone = -1;
		bool has_before = false;
		Transform3D before;
		real_t tilt = 0.0;
		real_t forearm_sink = 0.0;
		real_t clip_forearm_sink = 0.0;
		bool has_pole = false;
		Vector3 pole;
	};
	struct Circle {
		bool valid = false;
		Vector3 centre;
		real_t radius = 0.0;
		Vector3 u;
		Vector3 v;
	};

	ObjectID skeleton_id;
	LocalVector<Bone> bones;
	LocalVector<Transform3D> carries;
	LocalVector<uint32_t> carried_on;
	uint32_t tick = 1;

	PackedInt32Array point_triangles;
	PackedVector3Array point_weights;
	PackedInt32Array corners;
	PackedVector3Array positions;
	PackedInt32Array vertex_bones;
	PackedFloat32Array vertex_weights;
	PackedVector3Array normals;
	int slots = 4;

	BodyProfile torso;
	bool has_torso = false;
	Bone torso_bone;
	Arm arms[2];
	Hand hands[2];
	LocalVector<int> lean_chain;

	int sections = 4;
	int sectors = 12;
	real_t profile_from = 0.1;
	real_t profile_to = 0.9;
	real_t reach_share = 0.995;
	real_t lean_reach = 0.97;
	real_t max_lean = 0.6109;
	int forearm_samples = 10;
	real_t forearm_tolerance = 0.005;
	real_t tilt_step = 0.0873;
	real_t max_tilt = 0.6109;
	real_t near_straight = 2.618;
	real_t near_folded = 0.785;
	real_t pole_out = 0.3;

	Vector3 posed_normal;

	Skeleton3D *_skeleton() const;
	Transform3D _carry_of(const Bone &p_bone, const Skeleton3D *p_skeleton) const;
	const Transform3D &_carry(int p_bone, const Skeleton3D *p_skeleton);
	Vector3 _pose(int p_point, const Skeleton3D *p_skeleton, Vector3 &r_normal);
	bool _carried_by(const Skeleton3D *p_skeleton, int p_bone, int p_root) const;
	Vector3 _leaned(const LocalVector<Vector3> &p_joints, const Vector3 &p_point, const Vector3 &p_axis, real_t p_angle) const;
	real_t _lean_needed(const Skeleton3D *p_skeleton, const Vector3 &p_shoulder, const Vector3 &p_target, real_t p_want, Vector3 &r_axis) const;
	Circle _elbow_circle(const Vector3 &p_shoulder, const Vector3 &p_elbow, const Vector3 &p_target, real_t p_upper, real_t p_fore) const;
	real_t _forearm_sink(real_t p_radius, const Transform3D &p_back, const Vector3 &p_elbow, const Vector3 &p_wrist) const;
	void _pole(int p_side, const Skeleton3D *p_skeleton);

protected:
	static void _bind_methods();

public:
	void set_tuning(const Dictionary &p_tuning);
	void set_skeleton(Skeleton3D *p_skeleton);
	void set_bone_count(int p_count);
	void set_bone(int p_bone, int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest_inverse);
	void set_points(const PackedInt32Array &p_point_triangles, const PackedVector3Array &p_point_weights, const PackedInt32Array &p_corners,
			const PackedVector3Array &p_positions, const PackedInt32Array &p_bones, const PackedFloat32Array &p_weights,
			const PackedVector3Array &p_normals, int p_slots);
	void set_torso(const Vector3 &p_from, const Vector3 &p_axis, const Vector3 &p_front, const PackedFloat32Array &p_profile,
			int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest_inverse);
	void clear_torso();
	void set_arm(int p_side, int p_shoulder, int p_elbow, int p_hand, real_t p_upper, real_t p_fore, real_t p_forearm_radius);
	void set_lean_chain(const PackedInt32Array &p_chain);

	void begin_tick();
	Vector3 posed(int p_point);
	Vector3 get_posed_normal() const { return posed_normal; }
	PackedVector3Array place(int p_side, int p_hand_point, int p_anchor_point);
	real_t aim(int p_side, const Vector3 &p_miss, int p_anchor_bone);
	void lean(const Vector3 &p_axis, real_t p_angle);
	void clear_forearm(int p_side);

	Transform3D get_local_target(int p_side) const;
	void set_local_target(int p_side, const Transform3D &p_target);
	bool is_reachable(int p_side) const;
	Vector3 get_lean_axis(int p_side) const;
	Vector3 get_point_in_hand(int p_side) const;
	real_t get_tilt(int p_side) const;
	real_t get_forearm_sink(int p_side) const;
	real_t get_clip_forearm_sink(int p_side) const;
	bool has_pole(int p_side) const;
	Vector3 get_pole(int p_side) const;
};
