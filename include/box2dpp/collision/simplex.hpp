// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/collision/shape_proxy.hpp>

namespace box2dpp
{
	class Simplex;

	/// Simplex vertex used in the GJK algorithm
	/// Represents a pair of support points from two shapes
	class SimplexVertex final
	{
	public:
		using index_type = ShapeProxy::index_type;

		/// Support point from shape A (in A's local space)
		Vec2 w_a;

		/// Support point from shape B (in A's local space after transform)
		Vec2 w_b;

		/// Minkowski difference: w_a - w_b
		Vec2 w;

		/// Barycentric coordinate for closest point on simplex
		float weight;

		/// Vertex index in shape A
		index_type index_a;

		/// Vertex index in shape B
		index_type index_b;
	};

	/// Used to warm start the GJK simplex. 
	/// If you call this function multiple times with nearby transforms this might improve performance. 
	/// Otherwise, you can zero initialize this.
	/// The distance cache must be initialized to zero on the first call.
	class SimplexCache final
	{
	public:
		using index_type = ShapeProxy::index_type;

		const static SimplexCache zero;

		enum class Type : std::uint16_t
		{
			UNINITIALIZED = 0,

			/// Single point (vertex)
			POINT = 1,

			/// Line segment (two vertices)
			LINE_SEGMENT = 2,

			/// Triangle (three vertices)
			TRIANGLE = 3
		};

		/// The number of stored simplex points
		Type type;

		/// The cached simplex indices on shape A
		index_type index_a[3];

		/// The cached simplex indices on shape B
		index_type index_b[3];
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr SimplexCache SimplexCache::zero = {.type = Type::UNINITIALIZED, .index_a = {}, .index_b = {}};

	/// GJK simplex (point, line segment, or triangle)
	/// Used to compute the closest points between two convex shapes
	class Simplex final
	{
	public:
		// using SimplexCache::Type;
		enum class Type : std::uint32_t
		{
			/// Single point (vertex)
			POINT = 1,

			/// Line segment (two vertices)
			LINE_SEGMENT = 2,

			/// Triangle (three vertices)
			TRIANGLE = 3
		};

		/// Current simplex vertices (up to 3)
		std::array<SimplexVertex, 3> vertices;

		/// Number of active vertices (1, 2, or 3)
		Type type;

		/// Create simplex from cache (warm start)
		[[nodiscard]] static auto create(const SimplexCache& cache, const ShapeProxy& proxy_a, const ShapeProxy& proxy_b) noexcept -> Simplex;

		/// Create cache from simplex
		[[nodiscard]] auto cache() const noexcept -> SimplexCache;

	private:
		/// Solve 1-simplex (point case)
		[[nodiscard]] auto solve1() const noexcept -> Vec2;

		/// Solve 2-simplex (line segment case)
		/// @return Search direction towards origin
		[[nodiscard]] auto solve2() noexcept -> Vec2;

		/// Solve 3-simplex (triangle case)
		/// @return Zero vector if origin is inside triangle
		[[nodiscard]] auto solve3() noexcept -> Vec2;

	public:
		[[nodiscard]] auto solve() noexcept -> Vec2;

		/// Compute the closest points on the two shapes
		/// @return Closest point on shape A & B
		auto compute_closest_points() const noexcept -> std::pair<Vec2, Vec2>;
	};
}
