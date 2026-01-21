// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/circle.hpp>

#include <numbers>

namespace box2dpp
{
	auto Circle::valid() const noexcept -> bool
	{
		// Check for positive radius and finite center
		return radius > 0.0f and box2dpp::valid(radius) and center.valid();
	}

	auto Circle::diameter() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Diameter = 2 * radius
		return 2.0f * radius;
	}

	auto Circle::circumference() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Circumference = 2 * π * radius
		return 2.0f * std::numbers::pi_v<float> * radius;
	}

	auto Circle::area() const noexcept -> float
	{
		BPP_ASSERT(valid());

		// Area = π * radius²
		return std::numbers::pi_v<float> * radius * radius;
	}

	auto Circle::in(const Vec2& point) const noexcept -> bool
	{
		BPP_ASSERT(valid());

		// Check if squared distance from center is less than or equal to squared radius
		return center.distance_squared(point) <= radius * radius;
	}
}
