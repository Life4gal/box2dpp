// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/transform.hpp>

#include <box2dpp/collision/cast.hpp>
#include <box2dpp/collision/simplex.hpp>

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

	/// Result of computing the distance between two line segments
	class SegmentDistanceOutput final
	{
	public:
		/// The closest point on the first segment
		Vec2 closest1;

		/// The closest point on the second segment
		Vec2 closest2;

		/// The barycentric coordinate on the first segment
		float fraction1;

		/// The barycentric coordinate on the second segment
		float fraction2;

		/// The squared distance between the closest points
		float distance_squared;

		/// Compute the distance between two line segments, clamping at the end points if needed.
		[[nodiscard]] static auto compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistanceOutput;

		/// Compute the distance between two line segments, clamping at the end points if needed.
		[[nodiscard]] static auto compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistanceOutput;
	};
}
