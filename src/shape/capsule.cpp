// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/capsule.hpp>

#include <algorithm>

namespace box2dpp
{
	auto Capsule::in(const Vec2 point) const noexcept -> bool
	{
		const auto r2 = radius * radius;

		if (center2 == center1)
		{
			// Capsule is really a circle
			return center1.distance_squared(point) <= r2;
		}

		const auto diff = center2 - center1;
		const auto diff_l2 = diff.length_squared();

		// Get closest point on capsule segment
		// c = center1 + t * d
		// dot(point - c, d) = 0
		// dot(point - center1 - t * d, d) = 0
		// t = dot(point - center1, d) / dot(d, d)
		const auto t = std::ranges::clamp((point - center1).dot(diff) / diff_l2, .0f, 1.f);
		const auto c = center1 + t * diff;

		// Is query point within radius around closest point?
		return point.distance_squared(c) <= radius;
	}
}
