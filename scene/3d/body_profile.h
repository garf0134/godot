/**************************************************************************/
/*  body_profile.h                                                        */
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

#include "core/math/math_funcs.h"
#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// A body part's skin round a bone's axis, for AdventureAI (its HandRest.Thigh,
// which keeps the same lookup in GDScript: change both): the axis from one
// joint to the next, its front, and the skin's distance from the axis per
// cell, `sections` along it by `sectors` round it, section-major. The seated
// hands' thighs and the body contacts' torso are each one.
struct BodyProfile {
	Vector3 from;
	Vector3 axis;
	Vector3 front;
	Vector3 side;
	LocalVector<real_t> profile;

	void set(const Vector3 &p_from, const Vector3 &p_axis, const Vector3 &p_front, const PackedFloat32Array &p_profile) {
		from = p_from;
		axis = p_axis;
		front = p_front;
		side = p_axis.normalized().cross(p_front);
		profile.resize(p_profile.size());
		for (int i = 0; i < p_profile.size(); i++) {
			profile[i] = p_profile[i];
		}
	}

	// (fraction along the axis, angle round it from the front, distance from it).
	Vector3 polar(const Vector3 &p_point) const {
		const Vector3 d = p_point - from;
		const real_t t = d.dot(axis) / axis.length_squared();
		const Vector3 across = d - axis * t;
		return Vector3(t, Math::atan2(across.dot(side), across.dot(front)), across.length());
	}

	// The skin's distance from the axis at fraction `p_t` along it and
	// `p_angle` round it, interpolated between cell centres; beyond the
	// profile, its end sections. 0 for a profile of the wrong size.
	real_t skin_at(real_t p_t, real_t p_angle, int p_sections, int p_sectors, real_t p_profile_from, real_t p_profile_to) const {
		if ((int)profile.size() < p_sections * p_sectors || p_sections <= 0 || p_sectors <= 0) {
			return 0.0;
		}
		const real_t s = CLAMP((p_t - p_profile_from) / (p_profile_to - p_profile_from) * p_sections - 0.5, (real_t)0.0, (real_t)(p_sections - 1));
		const real_t a = Math::fposmod(p_angle / (real_t)Math::TAU * p_sectors, (real_t)p_sectors);
		const int s0 = (int)s;
		const int s1 = MIN(s0 + 1, p_sections - 1);
		const int a0 = (int)a % p_sectors;
		const int a1 = (a0 + 1) % p_sectors;
		const real_t fs = s - s0;
		const real_t fa = a - (int)a;
		const real_t near = Math::lerp(profile[s0 * p_sectors + a0], profile[s0 * p_sectors + a1], fa);
		const real_t far = Math::lerp(profile[s1 * p_sectors + a0], profile[s1 * p_sectors + a1], fa);
		return Math::lerp(near, far, fs);
	}
};
