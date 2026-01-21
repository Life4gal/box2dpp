// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/segment.hpp>

#include <algorithm>
#include <limits>
#include <ranges>

namespace box2dpp
{
	auto Segment::valid() const noexcept -> bool
	{
		// Check for finite endpoints
		// Note: zero-length segments are technically valid
		return point1.valid() and point2.valid();
	}

	auto Segment::length() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Euclidean distance between endpoints
		return point1.distance(point2);
	}

	auto Segment::length_squared() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Squared distance
		return point1.distance_squared(point2);
	}

	auto Segment::direction() const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		if (point1 == point2)
		{
			// Return zero vector for degenerate segment
			return {.x = 0.0f, .y = 0.0f};
		}

		// Normalized vector from point1 to point2
		return (point2 - point1).normalize();
	}

	auto Segment::midpoint() const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		// Average of the two endpoints
		return (point1 + point2) * 0.5f;
	}

	auto Segment::point_at(const float t) const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		// Linear interpolation between endpoints
		// t=0 -> point1, t=1 -> point2
		const auto clamped_t = std::ranges::clamp(t, 0.0f, 1.0f);
		return {.x = point1.x + clamped_t * (point2.x - point1.x), .y = point1.y + clamped_t * (point2.y - point1.y)};
	}

	auto Segment::closest_point(const Vec2& point) const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		// Project point onto line containing segment, then clamp to segment
		const auto t = project(point);
		return point_at(t);
	}

	auto Segment::distance_squared_to(const Vec2& point) const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Squared distance from point to the closest point on segment
		const auto closest = closest_point(point);
		return point.distance_squared(closest);
	}

	auto Segment::project(const Vec2& point) const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Compute barycentric coordinate t for closest point on line-segment
		const auto diff = point2 - point1;
		const auto diff_l2 = diff.length_squared();

		if (diff_l2 < std::numeric_limits<float>::epsilon())
		{
			// Degenerate segment (zero length), any t returns same point
			return 0.0f;
		}

		// t = ((point - point1)·diff) / (diff·diff)
		const auto t = (point - point1).dot(diff) / diff_l2;

		// Clamp to [0, 1] for segment, not infinite line
		return std::ranges::clamp(t, 0.0f, 1.0f);
	}

	auto SegmentDistance::compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistance
	{
		return compute(segment1.point1, segment1.point2, segment2.point1, segment2.point2);
	}

	auto SegmentDistance::compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistance
	{
		// Direction of first segment
		const auto d1 = q1 - p1;
		// Direction of second segment
		const auto d2 = q2 - p2;
		// Vector from p2 to p1
		const auto r = p1 - p2;

		// Squared length of first segment
		const auto dd1 = d1.dot(d1);
		// Squared length of second segment
		const auto dd2 = d2.dot(d2);
		// Projection of r onto d1
		const auto rd1 = r.dot(d1);
		// Projection of r onto d2
		const auto rd2 = r.dot(d2);

		constexpr auto epsilon2 = std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon();

		// Barycentric coordinate on first segment
		float fraction1;
		// Barycentric coordinate on second segment
		float fraction2;

		// Handle degenerate cases (zero-length segments)
		if (dd1 < epsilon2 or dd2 < epsilon2)
		{
			if (dd1 >= epsilon2)
			{
				// Segment 2 is degenerate, treat as point
				// Project p2 onto segment 1
				fraction1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				fraction2 = 0;
			}
			else if (dd2 >= epsilon2)
			{
				// Segment 1 is degenerate, treat as point
				// Project p1 onto segment 2
				fraction1 = 0;
				fraction2 = std::ranges::clamp(rd2 / dd2, 0.f, 1.f);
			}
			else
			{
				// Both segments are degenerate (both points)
				fraction1 = 0;
				fraction2 = 0;
			}
		}
		else
		{
			// Both segments have non-zero length
			// Use the closest points between two lines algorithm

			// Dot product of directions
			const auto d12 = d1.dot(d2);
			const auto denominator = dd1 * dd2 - d12 * d12;

			// Fraction on segment 1
			float f1 = 0;
			if (denominator != 0) // NOLINT(clang-diagnostic-float-equal)
			{
				// Segments are not parallel
				// Solve for optimal f1 (closest point on infinite line 1)
				f1 = std::ranges::clamp((d12 * rd2 - dd2 * rd1) / denominator, 0.f, 1.f);
			}
			// else: segments are parallel, f1 remains 0

			// Compute corresponding f2 for the current f1
			auto f2 = (d12 * f1 + rd2) / dd2;

			// If f2 is outside [0,1], clamp it and recompute f1
			if (f2 < 0)
			{
				// Clamp f2 to 0 and find best f1
				f1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				f2 = 0;
			}
			else if (f2 > 1)
			{
				// Clamp f2 to 1 and find best f1
				f1 = std::ranges::clamp((d12 - rd1) / dd1, 0.f, 1.f);
				f2 = 1;
			}

			fraction1 = f1;
			fraction2 = f2;
		}

		// Compute the closest points using barycentric coordinates
		const auto closest1 = multiply_add(p1, fraction1, d1);
		const auto closest2 = multiply_add(p2, fraction2, d2);
		const auto distance_squared = closest1.distance_squared(closest2);

		return {.closest1 = closest1, .closest2 = closest2, .fraction1 = fraction1, .fraction2 = fraction2, .distance_squared = distance_squared};
	}

	auto SegmentDistance::valid() const noexcept -> bool
	{
		// Check for finite values and non-negative squared distance
		return
				distance_squared >= 0.0f and
				box2dpp::valid(distance_squared) and
				closest1.valid() and
				closest2.valid() and
				box2dpp::valid(fraction1) and
				box2dpp::valid(fraction2);
	}

	auto SegmentDistance::distance() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Convert squared distance to actual distance
		return std::sqrt(distance_squared);
	}

	auto ChainSegment::valid() const noexcept -> bool
	{
		// Check all components are valid
		return ghost1.valid() and ghost2.valid() and segment.valid();
	}
}
