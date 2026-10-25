/**************************************************************************/
/*  drape_driver_modifier_3d.h                                            */
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

#include "scene/3d/skeleton_modifier_3d.h"

class DrapeRingsModifier3D;

// Turns each spring chain's (column's) top joint with the thighs before the
// springs swing the rest, lays the skirt over a seat, and hands a shader the
// legs' capsules to push the cloth out of: AdventureAI's draped garments
// (docs/DRAPED_GARMENTS.md §6.2, §6.3, §6.7 there). The layout and the seat's
// reading are a script's; the pass is here, since a script's per-pass cost
// was a third of the wearer's budget.
class DrapeDriverModifier3D : public SkeletonModifier3D {
	GDCLASS(DrapeDriverModifier3D, SkeletonModifier3D);

	// Per thigh: bone, rest rotation, direction towards its knee.
	LocalVector<int> thighs;
	LocalVector<Quaternion> thigh_rests;
	LocalVector<Vector3> thigh_along;

	// Per column, in the hips' frame.
	LocalVector<int> tops;
	LocalVector<Quaternion> rests;
	PackedVector2Array shares;
	PackedVector2Array weights;
	LocalVector<Vector3> downs;
	LocalVector<Vector3> outs;
	LocalVector<Quaternion> seat_turns;
	LocalVector<float> laps;
	Vector3 up = Vector3(0, 1, 0);

	// Tuning, the script's constants.
	double max_stretch = 1.05;
	double reversed_share = 0.5;
	double seat_lap_lift = 0.0;
	double hold_span = 0.1;
	double hold_time = 0.15;

	// Per leg capsule: bone, its ends in that bone's frame, its push radii.
	LocalVector<int> leg_bones;
	LocalVector<Vector3> leg_ends;
	LocalVector<Vector2> leg_radii;
	LocalVector<StringName> leg_params;
	PackedVector4Array capsules;

	ObjectID rings_id;
	ObjectID springs_id;
	ObjectID wardrobe_id;
	bool reset_springs = false;

	// Read and written by the script; recorded by the pass.
	double seated = 0.0;
	double hold = 0.0;
	PackedVector2Array applied;

	static Quaternion _turn(const Quaternion &p_swing, float p_weight);
	Quaternion _swing(Skeleton3D *p_skeleton, int p_k) const;
	float _outward_weight(const Quaternion &p_swing, int p_c, float p_weight) const;
	void _push_legs(Skeleton3D *p_skeleton);

protected:
	static void _bind_methods();
	virtual void _process_modification(double p_delta) override;

public:
	void set_layout(const PackedInt32Array &p_thighs, const PackedVector4Array &p_thigh_rests, const PackedVector3Array &p_thigh_along,
			const PackedInt32Array &p_tops, const PackedVector4Array &p_rests, const PackedVector2Array &p_shares, const PackedVector2Array &p_weights,
			const PackedVector3Array &p_downs, const PackedVector3Array &p_outs, const PackedVector4Array &p_seat_turns, const PackedFloat32Array &p_laps,
			const Vector3 &p_up);
	void clear_layout();
	void set_tuning(double p_max_stretch, double p_reversed_share, double p_seat_lap_lift, double p_hold_span, double p_hold_time);
	void set_legs_layout(const PackedInt32Array &p_bones, const PackedVector3Array &p_ends, const PackedVector2Array &p_radii);

	void set_rings_node(Node *p_rings);
	void set_springs_node(Node *p_springs);
	void set_wardrobe_node(Node *p_wardrobe);
	void reset_springs_in_pass();

	PackedVector4Array compute_leg_capsules(Skeleton3D *p_skeleton, Node *p_wardrobe);

	void set_seated(double p_seated);
	double get_seated() const;
	double get_hold() const;
	PackedVector2Array get_weights() const;
	PackedVector2Array get_applied() const;
};
