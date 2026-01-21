// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/capsule.hpp>

#include <algorithm>

namespace box2dpp
{
	auto Capsule::valid() const noexcept -> bool
	{
		// Check for valid radius and finite points
		return radius > 0.0f and box2dpp::valid(radius) and center1.valid() and center2.valid();
	}

	auto Capsule::length() const noexcept -> float
	{
		// Distance between the two centers
		return center1.distance(center2);
	}

	auto Capsule::direction() const noexcept -> Vec2
	{
		if (center1 == center2)
		{
			// Default direction for zero-length capsule
			return {.x = 1.0f, .y = 0.0f};
		}

		// Vector from center1 to center2, normalized
		return (center2 - center1).normalize();
	}

	auto Capsule::in(const Vec2& point) const noexcept -> bool
	{
		const auto r2 = radius * radius;

		// Capsule is effectively a circle when centers are the same
		if (center2 == center1)
		{
			return center1.distance_squared(point) <= r2;
		}

		// Compute vector between capsule centers and its squared length
		const auto diff = center2 - center1;
		const auto diff_l2 = diff.length_squared();

		// Find closest point on capsule's central line segment using projection
		// t = clamp((point - center1)·diff / diff·diff, 0, 1)
		const auto t = std::ranges::clamp((point - center1).dot(diff) / diff_l2, .0f, 1.f);
		// Closest point on line-segment
		const auto c = center1 + t * diff;

		// Check if point is within radius of the closest point
		return point.distance_squared(c) <= radius;
	}
}
