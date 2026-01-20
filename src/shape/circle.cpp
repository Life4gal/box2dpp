// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/circle.hpp>

namespace box2dpp
{
	auto Circle::in(const Vec2 point) const noexcept -> bool
	{
		return center.distance_squared(point) <= radius * radius;
	}
}
