// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A solid capsule can be viewed as two semicircles connected by a rectangle.
	class Capsule final
	{
	public:
		/// Local center of the first semicircle (start of capsule)
		Vec2 center1;

		/// Local center of the second semicircle (end of capsule)
		Vec2 center2;

		/// Radius of the semicircles and thickness of the rectangle
		float radius;

		/// Check if the capsule is valid (positive radius, finite points)
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Get the length of the capsule (distance between centers)
		[[nodiscard]] auto length() const noexcept -> float;

		/// Get the direction vector from center1 to center2
		[[nodiscard]] auto direction() const noexcept -> Vec2;

		/// Test if a point is inside the capsule in local space
		/// @param point The point to test in local coordinates
		/// @return true if the point is inside or on the boundary
		[[nodiscard]] auto in(const Vec2& point) const noexcept -> bool;
	};
}
