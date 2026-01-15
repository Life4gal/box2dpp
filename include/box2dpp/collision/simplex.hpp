// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	class ShapeProxy;

	class Simplex;

	/// Used to warm start the GJK simplex. 
	/// If you call this function multiple times with nearby transforms this might improve performance. 
	/// Otherwise, you can zero initialize this.
	/// The distance cache must be initialized to zero on the first call.
	/// Users should generally just zero initialize this structure for each call.
	class SimplexCache final
	{
	public:
		/// The number of stored simplex points
		std::uint16_t count;

		/// The cached simplex indices on shape A
		std::uint8_t index_a[3];

		/// The cached simplex indices on shape B
		std::uint8_t index_b[3];

		[[nodiscard]] static auto create(const Simplex& simplex) noexcept -> SimplexCache;
	};

	/// Simplex vertex for debugging the GJK algorithm
	class SimplexVertex final
	{
	public:
		/// support point in proxy_a
		Vec2 w_a;

		/// support point in proxy_b
		Vec2 w_b;

		/// w_b - w_a
		Vec2 w;

		/// barycentric coordinate for closest point
		float a;

		/// w_a index
		std::uint16_t index_a;

		/// w_b index
		std::uint16_t index_b;
	};

	/// Simplex from the GJK algorithm
	class Simplex final
	{
	public:
		/// vertices
		SimplexVertex vertices[3];

		/// number of valid vertices
		std::uint32_t count;

		[[nodiscard]] static auto create(const SimplexCache& cache, const ShapeProxy& proxy_a, const ShapeProxy& proxy_b) noexcept -> Simplex;

		// Solve a line segment using barycentric coordinates.
		//
		// p = a1 * w1 + a2 * w2
		// a1 + a2 = 1
		//
		// The vector from the origin to the closest point on the line is perpendicular to the line.
		// e12 = w2 - w1
		// dot(p, e) = 0
		// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
		//
		// 2-by-2 linear system
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		//
		// Define
		// d12_1 =  dot(w2, e12)
		// d12_2 = -dot(w1, e12)
		// d12 = d12_1 + d12_2
		//
		// Solution
		// a1 = d12_1 / d12
		// a2 = d12_2 / d12
		//
		// returns a vector that points towards the origin
		[[nodiscard]] auto solve2() noexcept -> Vec2;

		[[nodiscard]] auto solve3() noexcept -> Vec2;

		auto compute_witness_points(Vec2& out_a, Vec2& out_b) const noexcept -> void;
	};
}
