/**************************************************************************/
/*  drape_cloth_modifier_3d.h                                             */
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

// A position-based cloth solve over a draped garment's chain joints:
// AdventureAI's draped garments (docs/DRAPED_GARMENTS.md §6.9 there). Each
// joint is a particle, the top of every column pinned where the pose put
// it, with a tip past each hem bone. Links run down each column (held to
// their length), round each ring and diagonally to the ring above (held
// from stretching, free to gather), and the body's capsules push out the
// particles and every link between them, so a leg cannot pass between two
// columns. A particle or link already inside a capsule in the rest pose
// (cloth modelled tight to a leg, or a column between the legs whose links
// to its neighbours cross a thigh) may stay that deep in it and is pushed
// only past that, so the cloth is never shoved off the shape it was
// modelled in; a link through the inner half of a capsule at rest is not
// collided with it at all, its particles alone kept out. Each joint is drawn
// toward its hang in the pose, strongly at the top and fading to the hem;
// gravity has the rest. It is simulated in the
// center bone's frame, so travel through the world does not swing it. The
// layout is built by a script.
class DrapeClothModifier3D : public SkeletonModifier3D {
	GDCLASS(DrapeClothModifier3D, SkeletonModifier3D);

public:
	enum LinkKind {
		LINK_COLUMN,
		LINK_RING,
		LINK_DIAGONAL,
	};

private:
	int center_bone = -1;

	// Per particle: its bone (-1 for a tip), the particle above it (-1
	// pinned), a tip's length, and its ring (0 at the top).
	LocalVector<int> bones;
	LocalVector<int> parents;
	LocalVector<float> tip_lengths;
	LocalVector<int> ring_of;
	int deepest = 1;
	// Columns over the particles, top to tip.
	LocalVector<int> column_start;
	LocalVector<int> column_size;
	struct Link {
		int a = 0;
		int b = 0;
		int kind = LINK_COLUMN;
		float rest = 0.0;
	};
	LocalVector<Link> links;
	struct Capsule {
		int bone = -1;
		Vector3 a;
		Vector3 b;
		float radius_a = 0.0;
		float radius_b = 0.0;
	};
	LocalVector<Capsule> capsules;
	// How deep each particle and each link sits in each capsule at rest
	// (particle or link major, a capsule's slot within it): it may stay that
	// deep. Measured with the link lengths.
	LocalVector<float> particle_allowance;
	LocalVector<float> link_allowance;
	// A particle held near a leg (two capsules, the thigh's and the
	// shin's): no further than tether_slack from where it hung at rest,
	// carried by whichever of the two bones it was nearer at rest (`bone`,
	// `local` in its frame, measured with the allowances); `target` is that
	// place in the pass's pose, in the center bone's frame.
	struct Tether {
		int particle = 0;
		int legs[2] = { 0, 0 };
		int bone = -1;
		Vector3 local;
		Vector3 target;
	};
	LocalVector<Tether> tethers;

	int iterations = 4;
	double top_pull = 1.0;
	double hem_pull = 0.15;
	double damping = 0.04;
	double gravity = 9.8;
	double ring_stiffness = 0.5;
	double shear_stiffness = 0.3;
	double thickness = 0.015;
	double snap_distance = 0.15;
	double max_speed = 0.0;
	bool keep_sides = false;
	double tether_slack = 0.05;
	double tether_stiffness = 1.0;

	// Pass state, in the center bone's frame.
	bool measured = false;
	bool started = false;
	int resets = 0;
	LocalVector<Vector3> x;
	LocalVector<Vector3> previous;
	LocalVector<Vector3> hang;
	// Each particle's hang at the last step, so a step can tell how far the
	// pose itself moved it.
	LocalVector<Vector3> hang_before;
	LocalVector<float> mass_inverse;
	LocalVector<Capsule> placed;
	LocalVector<Quaternion> input_rotation;
	LocalVector<Vector3> input_origin;
	// Per column, its top bone's parent's rotation in the pose.
	LocalVector<Quaternion> root_parent_rotation;

	bool _read(Skeleton3D *p_skeleton, const Transform3D &p_center);
	void _step(const Transform3D &p_center, double p_delta);
	void _collide(int p_particle, const Capsule &p_capsule, float p_allowance);
	void _collide_link(int p_a, int p_b, const Capsule &p_capsule, float p_allowance);
	static float _depth(const Vector3 &p_a, const Vector3 &p_b, const Capsule &p_capsule);
	static Vector3 _on_axis(const Vector3 &p_point, const Capsule &p_capsule);
	void _write(Skeleton3D *p_skeleton, const Transform3D &p_center);

protected:
	static void _bind_methods();
	virtual void _process_modification(double p_delta) override;

public:
	void set_layout(int p_center_bone, const PackedInt32Array &p_bones, const PackedInt32Array &p_parents, const PackedFloat32Array &p_tip_lengths, const PackedInt32Array &p_links);
	void set_colliders(const PackedInt32Array &p_bones, const PackedVector3Array &p_ends, const PackedVector2Array &p_radii);
	void set_tethers(const PackedInt32Array &p_particles, const PackedInt32Array &p_legs);
	void clear_layout();
	void reset();

	void set_iterations(int p_iterations);
	int get_iterations() const;
	void set_top_pull(double p_pull);
	double get_top_pull() const;
	void set_hem_pull(double p_pull);
	double get_hem_pull() const;
	void set_damping(double p_damping);
	double get_damping() const;
	void set_gravity(double p_gravity);
	double get_gravity() const;
	void set_ring_stiffness(double p_stiffness);
	double get_ring_stiffness() const;
	void set_shear_stiffness(double p_stiffness);
	double get_shear_stiffness() const;
	void set_thickness(double p_thickness);
	double get_thickness() const;
	void set_snap_distance(double p_distance);
	double get_snap_distance() const;
	void set_max_speed(double p_speed);
	double get_max_speed() const;
	void set_keep_sides(bool p_keep);
	bool is_keeping_sides() const;
	void set_tether_slack(double p_slack);
	double get_tether_slack() const;
	void set_tether_stiffness(double p_stiffness);
	double get_tether_stiffness() const;

	int get_resets() const;
	PackedVector3Array get_particles() const;
};

VARIANT_ENUM_CAST(DrapeClothModifier3D::LinkKind);
