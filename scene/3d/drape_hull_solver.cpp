/**************************************************************************/
/*  drape_hull_solver.cpp                                                 */
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

#include "drape_hull_solver.h"

#include "core/math/geometry_3d.h"
#include "core/object/class_db.h"
#include "core/templates/hash_set.h"
#include "core/templates/pair.h"

// How much further a face turned right away from the side a point came from
// is counted (_push_out()), as a share of how deep the point is under its
// nearest face: near a piece's axis the side decides, and at its skin, where
// that depth is nothing, the nearest face does.
static constexpr real_t SIDE_BIAS = 1.0;

void DrapeHullSolver::set_hull(const PackedVector4Array &p_planes, const PackedInt32Array &p_piece_ends, const PackedByteArray &p_outer) {
	planes.clear();
	pieces.clear();
	any_outer = false;
	for (const Vector4 &v : p_planes) {
		planes.push_back(Plane(Vector3(v.x, v.y, v.z), v.w));
	}
	int start = 0;
	for (int k = 0; k < p_piece_ends.size(); k++) {
		Piece piece;
		piece.start = start;
		piece.end = MIN(p_piece_ends[k], (int)planes.size());
		piece.outer = k < p_outer.size() && p_outer[k] != 0;
		any_outer = any_outer || piece.outer;
		start = piece.end;
		// A sphere round the piece's corners: a point outside it is outside
		// the piece without reading its planes.
		Vector<Vector3> corners = piece.end > piece.start
				? Geometry3D::compute_convex_mesh_points(planes.ptr() + piece.start, piece.end - piece.start)
				: Vector<Vector3>();
		if (corners.is_empty()) {
			piece.radius = Math::INF;
		} else {
			Vector3 centre;
			for (const Vector3 &c : corners) {
				centre += c;
			}
			centre /= corners.size();
			real_t radius = 0.0;
			for (const Vector3 &c : corners) {
				radius = MAX(radius, centre.distance_to(c));
			}
			piece.centre = centre;
			piece.radius = radius + skin;
		}
		pieces.push_back(piece);
	}
}

void DrapeHullSolver::set_seat(const Vector3 &p_up, real_t p_level, const PackedVector3Array &p_footprint, const PackedFloat32Array &p_reaches) {
	has_seat = true;
	seat_up = p_up.normalized();
	seat_level = p_level;
	footprint_from.clear();
	footprint_to.clear();
	footprint_reach.clear();
	for (int k = 0; k + 1 < p_footprint.size() && (k >> 1) < p_reaches.size(); k += 2) {
		// Flattened onto the plane: the footprint is where the body is above
		// the seat, whatever height.
		footprint_from.push_back(p_footprint[k] - seat_up * p_footprint[k].dot(seat_up));
		footprint_to.push_back(p_footprint[k + 1] - seat_up * p_footprint[k + 1].dot(seat_up));
		footprint_reach.push_back(p_reaches[k >> 1]);
	}
}

void DrapeHullSolver::set_floor(real_t p_level) {
	floor_level = p_level;
}

void DrapeHullSolver::clear_seat() {
	has_seat = false;
	floor_level = -Math::INF;
	footprint_from.clear();
	footprint_to.clear();
	footprint_reach.clear();
}

void DrapeHullSolver::set_tuning(real_t p_skin, real_t p_seat_skin, int p_inner_passes, int p_outer_passes) {
	skin = p_skin;
	seat_skin = p_seat_skin;
	inner_passes = MAX(p_inner_passes, 1);
	outer_passes = MAX(p_outer_passes, 1);
}

void DrapeHullSolver::set_carries(const Array &p_carries) {
	carries.clear();
	for (int k = 0; k < p_carries.size(); k++) {
		carries.push_back(p_carries[k]);
	}
}

PackedVector3Array DrapeHullSolver::get_centres() const {
	PackedVector3Array out;
	for (const Piece &piece : pieces) {
		out.push_back(piece.centre);
	}
	return out;
}

PackedFloat32Array DrapeHullSolver::get_radii() const {
	PackedFloat32Array out;
	for (const Piece &piece : pieces) {
		out.push_back(piece.radius);
	}
	return out;
}

bool DrapeHullSolver::_near(const Piece &p_piece, const Vector3 &p_point, real_t p_margin) const {
	return p_point.distance_to(p_piece.centre) <= p_piece.radius + p_margin;
}

bool DrapeHullSolver::_contains(const Piece &p_piece, const Vector3 &p_point, real_t p_tolerance) const {
	for (int i = p_piece.start; i < p_piece.end; i++) {
		if (planes[i].distance_to(p_point) > p_tolerance) {
			return false;
		}
	}
	return true;
}

// The nearest point to p_point inside the piece: Dykstra's alternating
// projection, which converges on the nearest point, not just any inside.
Vector3 DrapeHullSolver::_project(const Piece &p_piece, const Vector3 &p_point) const {
	const int count = p_piece.end - p_piece.start;
	LocalVector<Vector3> corrections;
	corrections.resize(count);
	for (int i = 0; i < count; i++) {
		corrections[i] = Vector3();
	}
	Vector3 x = p_point;
	for (int pass = 0; pass < outer_passes; pass++) {
		bool settled = true;
		for (int i = 0; i < count; i++) {
			const Plane &plane = planes[p_piece.start + i];
			const Vector3 y = x + corrections[i];
			const real_t d = plane.distance_to(y);
			const Vector3 projected = d > 0.0 ? y - plane.normal * d : y;
			if (!projected.is_equal_approx(x)) {
				settled = false;
			}
			corrections[i] = y - projected;
			x = projected;
		}
		if (settled) {
			break;
		}
	}
	return x;
}

bool DrapeHullSolver::inside_inner(const Vector3 &p_point, real_t p_tolerance) const {
	for (const Piece &piece : pieces) {
		if (!piece.outer && _near(piece, p_point, 0.0) && _contains(piece, p_point, -p_tolerance)) {
			return true;
		}
	}
	return false;
}

bool DrapeHullSolver::inside_outer(const Vector3 &p_point, real_t p_tolerance) const {
	if (!any_outer) {
		return true;
	}
	for (const Piece &piece : pieces) {
		if (piece.outer && _near(piece, p_point, p_tolerance) && _contains(piece, p_point, p_tolerance)) {
			return true;
		}
	}
	return false;
}

// Out of every inner piece: through the nearest face of any piece the point
// is in whose far side is in no other, so overlapping pieces (thigh and shin
// at the knee) are left as one solid, not bounced between. Where every such
// face leads into another piece, through the nearest face, and again from
// there.
Vector3 DrapeHullSolver::push_out(const Vector3 &p_point) const {
	return _push_out(p_point, nullptr);
}

// push_out(), but with p_rest (the point's rest place, given set_carries())
// a point leaves each piece by the face nearest it, counting a face as
// further the more it is turned from where that piece carries the point's
// rest place: in a thigh's piece, near its axis, faces either side are
// about as near, and neighbouring cloth took opposite ones, a row out past
// the thigh and the next between the legs.
Vector3 DrapeHullSolver::_push_out(const Vector3 &p_point, const Vector3 *p_rest) const {
	Vector3 p = p_point;
	for (int pass = 0; pass < inner_passes; pass++) {
		Vector3 best = p;
		real_t best_distance = Math::INF;
		Vector3 fallback = p;
		real_t fallback_distance = Math::INF;
		for (uint32_t k = 0; k < pieces.size(); k++) {
			const Piece &piece = pieces[k];
			if (piece.outer || !_near(piece, p, 0.0) || !_contains(piece, p, 0.0)) {
				continue;
			}
			Vector3 way;
			real_t shallowest = 0.0;
			if (p_rest != nullptr && k < carries.size()) {
				way = carries[k].xform(*p_rest) - p;
				const real_t length = way.length();
				way = length > CMP_EPSILON ? way / length : Vector3();
				shallowest = Math::INF;
				for (int i = piece.start; i < piece.end; i++) {
					shallowest = MIN(shallowest, -planes[i].distance_to(p));
				}
			}
			for (int i = piece.start; i < piece.end; i++) {
				const real_t depth = planes[i].distance_to(p);
				const Vector3 q = p + planes[i].normal * (skin - depth);
				const real_t distance = -depth + (way == Vector3() ? 0.0 : SIDE_BIAS * shallowest * 0.5 * (1.0 - planes[i].normal.dot(way)));
				if (distance < fallback_distance) {
					fallback_distance = distance;
					fallback = q;
				}
				if (distance < best_distance && !inside_inner(q, 0.0)) {
					best_distance = distance;
					best = q;
				}
			}
		}
		if (fallback_distance == Math::INF) {
			break;
		}
		if (best_distance < Math::INF) {
			return best;
		}
		p = fallback;
	}
	return p;
}

// The point, or the nearest point to it in the nearest outer piece when it
// is in none; nearest first by each piece's sphere, so a piece whose sphere
// is further than the best projection found is never projected onto.
Vector3 DrapeHullSolver::pull_in(const Vector3 &p_point) const {
	if (inside_outer(p_point, 0.0)) {
		return p_point;
	}
	LocalVector<Pair<real_t, int>> order;
	for (uint32_t k = 0; k < pieces.size(); k++) {
		if (pieces[k].outer) {
			order.push_back(Pair<real_t, int>(p_point.distance_to(pieces[k].centre) - pieces[k].radius, (int)k));
		}
	}
	order.sort_custom<PairSort<real_t, int>>();
	Vector3 best = p_point;
	real_t best_distance = Math::INF;
	for (const Pair<real_t, int> &entry : order) {
		if (entry.first > 0.0 && entry.first * entry.first >= best_distance) {
			break;
		}
		const Vector3 q = _project(pieces[entry.second], p_point);
		const real_t distance = q.distance_squared_to(p_point);
		if (distance < best_distance) {
			best_distance = distance;
			best = q;
		}
	}
	return best;
}

// Into the outer union, then out of the inner one: the inner wins where
// they disagree, so cloth is never left inside the body.
Vector3 DrapeHullSolver::clamp_point(const Vector3 &p_point) const {
	return push_out(pull_in(p_point));
}

bool DrapeHullSolver::in_footprint(const Vector3 &p_point) const {
	const Vector3 flat = p_point - seat_up * p_point.dot(seat_up);
	for (uint32_t k = 0; k < footprint_from.size(); k++) {
		const Vector3 a = footprint_from[k];
		const Vector3 ab = footprint_to[k] - a;
		const real_t t = CLAMP((flat - a).dot(ab) / MAX(ab.length_squared(), (real_t)1e-9), 0.0, 1.0);
		if (flat.distance_to(a + ab * t) <= footprint_reach[k]) {
			return true;
		}
	}
	return false;
}

bool DrapeHullSolver::under_seat(const Vector3 &p_point, real_t p_tolerance) const {
	if (!has_seat) {
		return false;
	}
	const real_t height = p_point.dot(seat_up);
	return height < floor_level - p_tolerance || (height < seat_level - p_tolerance && in_footprint(p_point));
}

// Lifted onto the seat where it is under it inside the footprint, and onto
// the floor anywhere under it.
Vector3 DrapeHullSolver::on_seat(const Vector3 &p_point) const {
	if (!has_seat) {
		return p_point;
	}
	const real_t height = p_point.dot(seat_up);
	if (height < floor_level + seat_skin) {
		return p_point + seat_up * (floor_level + seat_skin - height);
	}
	if (height >= seat_level + seat_skin || !in_footprint(p_point)) {
		return p_point;
	}
	return p_point + seat_up * (seat_level + seat_skin - height);
}

// Into the band, then onto the seat: the seat wins, as cloth cannot pass
// through it whatever the hull says.
Vector3 DrapeHullSolver::settle(const Vector3 &p_point) const {
	return _settle(p_point, nullptr);
}

Vector3 DrapeHullSolver::_settle(const Vector3 &p_point, const Vector3 *p_rest) const {
	return on_seat(_push_out(pull_in(p_point), p_rest));
}

// The cloth from p_start (one point per vertex) settled, then kept whole:
// up to p_passes passes holding each edge between two fully hanging
// vertices (p_hanging 1) between its length in p_rest and p_max_stretch
// times it, each re-settling what an edge moved, ending once no edge is
// further than p_settled from its allowed length. Vertices that do not
// hang (p_hanging 0) are returned as given. Counts the faults: hanging
// vertices left more than p_fault inside the inner union or under the seat.
PackedVector3Array DrapeHullSolver::solve(const PackedVector3Array &p_rest, const PackedVector3Array &p_start, const PackedFloat32Array &p_hanging,
		const PackedInt32Array &p_indices, real_t p_max_stretch, int p_passes, real_t p_settled, real_t p_fault) {
	faults = 0;
	passes_run = 0;
	largest = 0.0;
	const int n = p_start.size();
	ERR_FAIL_COND_V(p_rest.size() != n || p_hanging.size() != n, p_start);
	PackedVector3Array out = p_start;
	Vector3 *moved = out.ptrw();
	const real_t *hanging = p_hanging.ptr();
	const Vector3 *rest = p_rest.ptr();
	for (int v = 0; v < n; v++) {
		if (hanging[v] > 0.0) {
			moved[v] = _settle(moved[v], rest + v);
		}
	}

	// Only edges between cloth that hangs fully: the band above it is the
	// shader's to blend, and held to its fitted neighbours here it pulled the
	// lap back up the band every pass.
	LocalVector<int> edges;
	LocalVector<real_t> lengths;
	HashSet<uint64_t> seen;
	const int *indices = p_indices.ptr();
	for (int t = 0; t + 2 < p_indices.size(); t += 3) {
		for (int k = 0; k < 3; k++) {
			const int a = indices[t + k];
			const int b = indices[t + (k + 1) % 3];
			if (a < 0 || b < 0 || a >= n || b >= n || hanging[a] < 1.0 || hanging[b] < 1.0) {
				continue;
			}
			const uint64_t key = ((uint64_t)MIN(a, b) << 32) | (uint64_t)MAX(a, b);
			if (seen.has(key)) {
				continue;
			}
			seen.insert(key);
			edges.push_back(a);
			edges.push_back(b);
			lengths.push_back(p_rest[a].distance_to(p_rest[b]));
		}
	}
	// And the faces between it, whose middles and edges' middles are held out
	// of the inner union as their corners are: stretched over a knee, a face
	// with every corner clear cut 6 cm into it.
	LocalVector<int> faces;
	for (int t = 0; t + 2 < p_indices.size(); t += 3) {
		const int a = indices[t];
		const int b = indices[t + 1];
		const int c = indices[t + 2];
		if (a >= 0 && b >= 0 && c >= 0 && a < n && b < n && c < n && hanging[a] >= 1.0 && hanging[b] >= 1.0 && hanging[c] >= 1.0) {
			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
		}
	}

	LocalVector<uint8_t> touched;
	touched.resize(n);
	for (int pass = 0; pass < p_passes; pass++) {
		passes_run = pass + 1;
		memset(touched.ptr(), 0, n);
		largest = 0.0;
		for (uint32_t e = 0; e < lengths.size(); e++) {
			const int a = edges[e * 2];
			const int b = edges[e * 2 + 1];
			const Vector3 d = moved[a] - moved[b];
			const real_t length = d.length();
			if (length < 1e-7) {
				continue;
			}
			const real_t wanted = CLAMP(length, lengths[e], lengths[e] * p_max_stretch);
			const real_t change = length - wanted;
			if (Math::abs(change) < p_settled) {
				continue;
			}
			const Vector3 correction = d * (change / length * 0.5);
			moved[a] -= correction;
			moved[b] += correction;
			touched[a] = 1;
			touched[b] = 1;
			largest = MAX(largest, Math::abs(change));
		}
		// A face's middle, or an edge's, inside the inner union moves its
		// corners out with it, by the side its rest place came from.
		for (uint32_t f = 0; f < faces.size(); f += 3) {
			const int corner[3] = { faces[f], faces[f + 1], faces[f + 2] };
			for (int k = 0; k < 4; k++) {
				// Each edge's middle, then the face's.
				const int i = k == 3 ? -1 : k;
				const int j = k == 3 ? -1 : (k + 1) % 3;
				Vector3 at;
				Vector3 from;
				if (i < 0) {
					at = (moved[corner[0]] + moved[corner[1]] + moved[corner[2]]) / 3.0;
					from = (rest[corner[0]] + rest[corner[1]] + rest[corner[2]]) / 3.0;
				} else {
					at = (moved[corner[i]] + moved[corner[j]]) * 0.5;
					from = (rest[corner[i]] + rest[corner[j]]) * 0.5;
				}
				if (!inside_inner(at, 0.0)) {
					continue;
				}
				const Vector3 by = _push_out(at, &from) - at;
				for (int m = 0; m < 3; m++) {
					if (i < 0 || m == i || m == j) {
						moved[corner[m]] += by;
						touched[corner[m]] = 1;
					}
				}
				largest = MAX(largest, by.length());
			}
		}
		// Re-settled only where an edge moved it: the rest are where the
		// last pass left them.
		for (int v = 0; v < n; v++) {
			if (touched[v] && hanging[v] > 0.0) {
				moved[v] = _settle(moved[v], rest + v);
			}
		}
		if (largest < p_settled) {
			break;
		}
	}
	for (int v = 0; v < n; v++) {
		if (hanging[v] > 0.0 && (inside_inner(moved[v], p_fault) || under_seat(moved[v], p_fault))) {
			faults++;
		}
	}
	return out;
}

void DrapeHullSolver::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_hull", "planes", "piece_ends", "outer"), &DrapeHullSolver::set_hull);
	ClassDB::bind_method(D_METHOD("set_seat", "up", "level", "footprint", "reaches"), &DrapeHullSolver::set_seat);
	ClassDB::bind_method(D_METHOD("clear_seat"), &DrapeHullSolver::clear_seat);
	ClassDB::bind_method(D_METHOD("set_floor", "level"), &DrapeHullSolver::set_floor);
	ClassDB::bind_method(D_METHOD("set_tuning", "skin", "seat_skin", "inner_passes", "outer_passes"), &DrapeHullSolver::set_tuning);
	ClassDB::bind_method(D_METHOD("set_carries", "carries"), &DrapeHullSolver::set_carries);
	ClassDB::bind_method(D_METHOD("get_centres"), &DrapeHullSolver::get_centres);
	ClassDB::bind_method(D_METHOD("get_radii"), &DrapeHullSolver::get_radii);
	ClassDB::bind_method(D_METHOD("clamp_point", "point"), &DrapeHullSolver::clamp_point);
	ClassDB::bind_method(D_METHOD("push_out", "point"), &DrapeHullSolver::push_out);
	ClassDB::bind_method(D_METHOD("pull_in", "point"), &DrapeHullSolver::pull_in);
	ClassDB::bind_method(D_METHOD("on_seat", "point"), &DrapeHullSolver::on_seat);
	ClassDB::bind_method(D_METHOD("settle", "point"), &DrapeHullSolver::settle);
	ClassDB::bind_method(D_METHOD("inside_inner", "point", "tolerance"), &DrapeHullSolver::inside_inner);
	ClassDB::bind_method(D_METHOD("inside_outer", "point", "tolerance"), &DrapeHullSolver::inside_outer);
	ClassDB::bind_method(D_METHOD("in_footprint", "point"), &DrapeHullSolver::in_footprint);
	ClassDB::bind_method(D_METHOD("under_seat", "point", "tolerance"), &DrapeHullSolver::under_seat);
	ClassDB::bind_method(D_METHOD("solve", "rest", "start", "hanging", "indices", "max_stretch", "passes", "settled", "fault"), &DrapeHullSolver::solve);
	ClassDB::bind_method(D_METHOD("get_faults"), &DrapeHullSolver::get_faults);
	ClassDB::bind_method(D_METHOD("get_passes_run"), &DrapeHullSolver::get_passes_run);
	ClassDB::bind_method(D_METHOD("get_largest"), &DrapeHullSolver::get_largest);
}
