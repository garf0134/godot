/**************************************************************************/
/*  seated_hands_solver.h                                                 */
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

class Skeleton3D;

// Rests a seated character's palms on its thighs and lays its fingers
// along them, for AdventureAI's seated hands (docs/SEATED_HANDS.md there):
// the target a hand moves the least distance to, against a body's measured
// thigh profile and palm, read from the animated skeleton's pose; and each
// finger joint's turn that sets the next joint its pad's thickness off the
// thigh. The scheduling, the weights and the tuning are a script's; the
// arithmetic is here, since a script's took 0.13 ms a sitter.
class SeatedHandsSolver : public RefCounted {
	GDCLASS(SeatedHandsSolver, RefCounted);

	struct Thigh {
		Vector3 from;
		Vector3 axis;
		Vector3 front;
		Vector3 side;
		LocalVector<real_t> profile;
	};
	struct Palm {
		Vector3 wrist;
		Vector3 knuckle;
		Vector3 normal;
		Vector3 point;
		LocalVector<Vector3> contacts;
		real_t finger_pad = 0.0;
	};
	// A bone read off the animated skeleton into the skinned one's space.
	struct Bone {
		int index = -1;
		Basis rest_turn;
		Transform3D skinned_rest;
		Transform3D skinned_rest_inverse;
		Basis to_general;
	};
	struct Fingers {
		int hand = -1;
		int middle = -1;
		int index = -1;
		int little = -1;
		// Three a finger, knuckle to tip.
		LocalVector<int> bones;
		LocalVector<real_t> angles;
		LocalVector<Vector3> axes;
		LocalVector<real_t> shown;
	};

	Thigh thighs[2];
	Palm palms[2];
	Bone thigh_bones[2];
	Bone hand_bones[2];
	Fingers fingers[2];
	ObjectID skeleton_id;

	int sections = 4;
	int sectors = 12;
	real_t profile_from = 0.1;
	real_t profile_to = 0.9;
	real_t contact_full = 0.18;
	real_t contact_none = 0.26;
	real_t max_side_angle = 0.5235988;
	real_t max_palm_roll = 0.4363323;
	real_t palm_clearance = 0.005;
	int resolve_passes = 8;
	real_t converged = 0.0002;
	real_t max_extend = 0.1745329;
	real_t max_flex = 1.5707963;
	real_t max_drape = 0.6108652;
	real_t relaxed_bend = 0.2617994;
	real_t tip_share = 0.8;
	int newton_steps = 6;
	real_t contact_tolerance = 0.001;

	real_t weight = 0.0;
	real_t palm_gap = Math::INF;
	real_t tilt = 0.0;
	real_t roll = 0.0;
	Transform3D hand;
	Transform3D ik_target;
	Transform3D move;
	Vector3 outward;

	Skeleton3D *_skeleton() const;
	Transform3D _carry(const Bone &p_bone, const Skeleton3D *p_skeleton) const;
	Vector3 _polar(const Thigh &p_thigh, const Vector3 &p_point) const;
	real_t _skin_at(const Thigh &p_thigh, real_t p_t, real_t p_angle) const;
	real_t _gap(const Thigh &p_thigh, const Vector3 &p_point) const;
	Vector3 _outward(const Thigh &p_thigh, const Vector3 &p_at) const;
	Transform3D _rest_move(const Thigh &p_thigh, const Vector3 &p_wrist, const Vector3 &p_line, const Vector3 &p_normal,
			const Vector3 &p_point, const LocalVector<Vector3> *p_contacts, const Vector3 &p_outward, real_t p_roll) const;
	void _left_to_clip(const Transform3D &p_hand_carry, const Transform3D &p_hand_rest);
	Vector2 _off(const Thigh &p_thigh, const Vector3 &p_joint, const Vector3 &p_segment, const Vector3 &p_axis, real_t p_pad, real_t p_angle) const;
	real_t _contact_angle(const Thigh &p_thigh, const Vector3 &p_joint, const Vector3 &p_segment, const Vector3 &p_axis,
			real_t p_pad, real_t p_low, real_t p_high, real_t p_bend) const;

protected:
	static void _bind_methods();

public:
	void set_tuning(const Dictionary &p_tuning);
	void set_thigh(int p_side, const Vector3 &p_from, const Vector3 &p_axis, const Vector3 &p_front, const PackedFloat32Array &p_profile);
	void set_palm(int p_side, const Vector3 &p_wrist, const Vector3 &p_knuckle, const Vector3 &p_normal, const Vector3 &p_point,
			const PackedVector3Array &p_contacts, real_t p_finger_pad);
	void set_skeleton(Skeleton3D *p_skeleton);
	void set_bone(int p_side, bool p_hand, int p_index, const Basis &p_rest_turn, const Transform3D &p_skinned_rest, const Basis &p_to_general);
	void set_finger_bones(int p_side, int p_hand, int p_middle, int p_index, int p_little, const PackedInt32Array &p_bones);

	real_t gap(int p_side, const Vector3 &p_point) const;
	Transform3D carry(int p_side, bool p_hand) const;
	Transform3D clip(int p_side) const;

	void solve_carried(int p_side, const Transform3D &p_thigh_carry, const Transform3D &p_hand_carry, const Transform3D &p_hand_rest,
			const Vector3 &p_warm);
	void reapply_carried(int p_side, const Transform3D &p_thigh_carry, const Transform3D &p_hand_carry, const Transform3D &p_hand_rest,
			const Transform3D &p_move);
	void solve(int p_side, bool p_full, const Transform3D &p_move, const Vector3 &p_warm);

	real_t get_weight() const { return weight; }
	real_t get_palm_gap() const { return palm_gap; }
	real_t get_tilt() const { return tilt; }
	real_t get_roll() const { return roll; }
	Transform3D get_hand() const { return hand; }
	Transform3D get_ik_target() const { return ik_target; }
	Transform3D get_move() const { return move; }
	Vector3 get_outward() const { return outward; }

	void search_fingers(int p_side);
	void ease_fingers(int p_side, real_t p_share, real_t p_limit);
	real_t lay_fingers(int p_side, real_t p_weight);
	void clear_fingers(int p_side);
	bool has_fingers(int p_side) const;
	bool has_finger_angles(int p_side) const;
	PackedFloat32Array get_finger_angles(int p_side) const;
	void set_finger_angles(int p_side, const PackedFloat32Array &p_angles);
	PackedFloat32Array get_finger_shown(int p_side) const;
};
