// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A solid circle
	class Circle final
	{
	public:
		/// The local center
		Vec2 center;

		/// The radius
		float radius;

		/// Test a point for overlap with a circle in local space
		[[nodiscard]] auto in(const Vec2 point) const noexcept -> bool
		{
			return center.distance_squared(point) <= radius * radius;
		}
	};
}
