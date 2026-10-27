/**************************************************************************/
/*  drape_hull_solver.h                                                   */
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

// Holds cloth between two unions of convex pieces and above a seat, for
// AdventureAI's seated drape targets (docs/DRAPED_GARMENTS.md §6.10 there):
// an inner union the cloth may not enter, an outer one it may not leave,
// and a plane it may not pass below inside a footprint. The pieces are
// fitted to a body by a script; the clamp and the edge passes are here,
// since a script's took 300 ms a garment.
class DrapeHullSolver : public RefCounted {
	GDCLASS(DrapeHullSolver, RefCounted);

	struct Piece {
		int start = 0;
		int end = 0;
		bool outer = false;
		Vector3 centre;
		real_t radius = 0.0;
	};
	LocalVector<Plane> planes;
	LocalVector<Piece> pieces;
	bool any_outer = false;

	bool has_seat = false;
	Vector3 seat_up = Vector3(0, 1, 0);
	real_t seat_level = 0.0;
	LocalVector<Vector3> footprint_from;
	LocalVector<Vector3> footprint_to;
	LocalVector<real_t> footprint_reach;

	real_t skin = 0.0005;
	real_t seat_skin = 0.003;
	int inner_passes = 4;
	int outer_passes = 24;

	int faults = 0;
	int passes_run = 0;
	real_t largest = 0.0;

	bool _contains(const Piece &p_piece, const Vector3 &p_point, real_t p_tolerance) const;
	bool _near(const Piece &p_piece, const Vector3 &p_point, real_t p_margin) const;
	Vector3 _project(const Piece &p_piece, const Vector3 &p_point) const;

protected:
	static void _bind_methods();

public:
	void set_hull(const PackedVector4Array &p_planes, const PackedInt32Array &p_piece_ends, const PackedByteArray &p_outer);
	void set_seat(const Vector3 &p_up, real_t p_level, const PackedVector3Array &p_footprint, const PackedFloat32Array &p_reaches);
	void clear_seat();
	void set_tuning(real_t p_skin, real_t p_seat_skin, int p_inner_passes, int p_outer_passes);

	PackedVector3Array get_centres() const;
	PackedFloat32Array get_radii() const;

	Vector3 clamp_point(const Vector3 &p_point) const;
	Vector3 push_out(const Vector3 &p_point) const;
	Vector3 pull_in(const Vector3 &p_point) const;
	Vector3 on_seat(const Vector3 &p_point) const;
	Vector3 settle(const Vector3 &p_point) const;
	bool inside_inner(const Vector3 &p_point, real_t p_tolerance) const;
	bool inside_outer(const Vector3 &p_point, real_t p_tolerance) const;
	bool in_footprint(const Vector3 &p_point) const;
	bool under_seat(const Vector3 &p_point, real_t p_tolerance) const;

	PackedVector3Array solve(const PackedVector3Array &p_rest, const PackedVector3Array &p_start, const PackedFloat32Array &p_hanging,
			const PackedInt32Array &p_indices, real_t p_max_stretch, int p_passes, real_t p_settled, real_t p_fault);
	int get_faults() const { return faults; }
	int get_passes_run() const { return passes_run; }
	real_t get_largest() const { return largest; }
};
