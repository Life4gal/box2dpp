// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A line segment with two-sided collision.
	class Segment final
	{
	public:
		/// The first point
		Vec2 point1;

		/// The second point
		Vec2 point2;
	};

	/// Result of computing the distance between two line segments
	class SegmentDistance final
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
		[[nodiscard]] static auto compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistance;

		/// Compute the distance between two line segments, clamping at the end points if needed.
		[[nodiscard]] static auto compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistance;
	};

	/// A line segment with one-sided collision. Only collides on the right side.
	/// Several of these are generated for a chain shape.
	/// ghost1 -> point1 -> point2 -> ghost2
	class ChainSegment final
	{
	public:
		/// The tail ghost vertex
		Vec2 ghost1;

		/// The line segment
		Segment segment;

		/// The head ghost vertex
		Vec2 ghost2;

		/// The owning chain shape index (internal usage only)
		// id_type chain_id;
	};
}
