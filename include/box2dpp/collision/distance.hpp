// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/transform.hpp>

#include <box2dpp/collision/simplex.hpp>
#include <box2dpp/collision/shape_cast.hpp>

namespace box2dpp
{
	class DistanceInput final
	{
	public:
		/// The proxy for shape A
		ShapeProxy proxy_a;

		/// The proxy for shape B
		ShapeProxy proxy_b;

		/// The world transform for shape A
		Transform transform_a;

		/// The world transform for shape B
		Transform transform_b;

		/// Should the proxy radius be considered?
		bool use_radii;
	};

	class DistanceOutput final
	{
	public:
		/// Closest point on shape A
		Vec2 point_a;

		/// Closest point on shape B
		Vec2 point_b;

		/// Normal vector that points from A to B. Invalid if distance is zero.
		Vec2 normal;

		/// The final distance, zero if overlapped
		float distance;

		/// Number of GJK iterations used
		std::uint32_t iterations;

		/// The number of simplexes stored in the simplex array
		std::uint32_t simplex_count;

		/// Compute the closest points between two shapes represented as point clouds.
		/// SimplexCache cache is input/output. On the first call set SimplexCache::count to zero.
		/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. 
		[[nodiscard]] static auto compute(const DistanceInput& input, SimplexCache& inout_cache, [[maybe_unused]] std::span<Simplex> debug_simplexes = {}) noexcept -> DistanceOutput;

		/// Compute the closest points between two shapes represented as point clouds.
		/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity.
		[[nodiscard]] static auto compute(const DistanceInput& input, [[maybe_unused]] std::span<Simplex> debug_simplexes = {}) noexcept -> DistanceOutput;
	};
}
