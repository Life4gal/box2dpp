// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A line segment with two-sided collision.
	/// Both sides of the segment can collide with other shapes.
	class Segment final
	{
	public:
		/// First endpoint of the segment
		Vec2 point1;

		/// Second endpoint of the segment
		Vec2 point2;

		/// Check if the segment is valid (finite endpoints, non-zero length)
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Get the length of the segment
		[[nodiscard]] auto length() const noexcept -> float;

		/// Get the squared length of the segment (faster than length())
		[[nodiscard]] auto length_squared() const noexcept -> float;

		/// Get the direction vector from point1 to point2 (normalized)
		[[nodiscard]] auto direction() const noexcept -> Vec2;

		/// Get the midpoint of the segment
		[[nodiscard]] auto midpoint() const noexcept -> Vec2;

		/// Get a point on the segment at parameter t (0 <= t <= 1)
		/// @param t Linear interpolation parameter (0 returns point1, 1 returns point2)
		[[nodiscard]] auto point_at(float t) const noexcept -> Vec2;

		/// Compute the closest point on this segment to a given point
		[[nodiscard]] auto closest_point(const Vec2& point) const noexcept -> Vec2;

		/// Compute the squared distance from this segment to a point
		[[nodiscard]] auto distance_squared_to(const Vec2& point) const noexcept -> float;

		/// Compute the barycentric coordinate of the closest point on segment
		/// @param point The point to project
		/// @return Parameter t where closest_point = point1 + t*(point2-point1)
		[[nodiscard]] auto project(const Vec2& point) const noexcept -> float;
	};

	/// Result of computing the distance between two line segments
	/// Contains closest points and barycentric coordinates
	class SegmentDistance final
	{
	public:
		/// Closest point on the first segment
		Vec2 closest1;

		/// Closest point on the second segment
		Vec2 closest2;

		/// Barycentric coordinate on the first segment (0 to 1)
		float fraction1;

		/// Barycentric coordinate on the second segment (0 to 1)
		float fraction2;

		/// Squared distance between the closest points
		float distance_squared;

		/// Compute the distance between two line segments, clamping at the end points if needed.
		[[nodiscard]] static auto compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistance;

		/// Compute the distance between two line segments, clamping at the end points if needed.
		[[nodiscard]] static auto compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistance;

		/// Check if the distance result is valid
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Get the actual distance (sqrt of distance_squared)
		[[nodiscard]] auto distance() const noexcept -> float;
	};

	/// A line segment with one-sided collision. Only collides on the right side.
	/// Several of these are generated for a chain shape.
	/// ghost1 -> point1 -> point2 -> ghost2
	class ChainSegment final
	{
	public:
		/// Previous vertex in chain (for normal computation)
		Vec2 ghost1;

		/// The actual line segment
		Segment segment;

		/// Next vertex in chain (for normal computation)
		Vec2 ghost2;

		/// The owning chain shape index (internal usage only)
		// id_type chain_id;

		/// Check if the chain segment is valid
		[[nodiscard]] auto valid() const noexcept -> bool;
	};
}
