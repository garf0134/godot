/**************************************************************************/
/*  drape_rings_modifier_3d.h                                             */
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

// Evens spring chains (columns) back out around rings about an axis, after
// the springs that move them: AdventureAI's draped garments
// (docs/DRAPED_GARMENTS.md §6.2 there). The layout is built by a script; the
// pass is here, since a script's per-pass cost was more than the springs'.
class DrapeRingsModifier3D : public SkeletonModifier3D {
	GDCLASS(DrapeRingsModifier3D, SkeletonModifier3D);

	int center_bone = -1;
	double max_stretch = 1.05;

	// Per panel: axis origin, down and e1, in the center bone's rest frame.
	LocalVector<Vector3> axes;
	// Every chain bone, column by column, top to bottom.
	LocalVector<int> bones;
	LocalVector<int> column_start;
	LocalVector<int> column_size;
	struct Ring {
		int panel = 0;
		int index = 0;
		int start = 0;
		int size = 0;
		bool closed = false;
		float rest = 0.0;
	};
	LocalVector<Ring> rings;
	LocalVector<int> members;
	LocalVector<float> rest_angles;

	// Records of the last drawn pass.
	PackedFloat32Array stretch;
	PackedByteArray held;
	double largest_turn = 0.0;

	// Pass state, sized by set_layout().
	LocalVector<Vector3> origin;
	LocalVector<Quaternion> rotation;
	LocalVector<Quaternion> after;
	LocalVector<uint8_t> turned;
	LocalVector<Quaternion> motion_rotation;
	LocalVector<Vector3> motion_origin;
	LocalVector<Vector3> at;
	LocalVector<float> offset;

	void _even(Skeleton3D *p_skeleton);

protected:
	static void _bind_methods();
	virtual void _process_modification(double p_delta) override;

public:
	void set_layout(int p_center_bone, const PackedVector3Array &p_axes, const PackedInt32Array &p_bones, const PackedInt32Array &p_column_start, const PackedInt32Array &p_column_size, const PackedInt32Array &p_rings, const PackedFloat32Array &p_ring_rest, const PackedInt32Array &p_members, const PackedFloat32Array &p_rest_angles);
	void clear_layout();

	void set_max_stretch(double p_max_stretch);
	double get_max_stretch() const;

	PackedFloat32Array get_stretch() const;
	PackedByteArray get_held() const;
	double get_largest_turn() const;
};
