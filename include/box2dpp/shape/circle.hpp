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
		/// Local center of the circle
		Vec2 center;

		/// Radius of the circle
		float radius;

		/// Check if the circle is valid (positive radius, finite center)
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Get the diameter of the circle
		[[nodiscard]] auto diameter() const noexcept -> float;

		/// Get the circumference (perimeter) of the circle
		[[nodiscard]] auto circumference() const noexcept -> float;

		/// Get the area of the circle
		[[nodiscard]] auto area() const noexcept -> float;

		/// Test if a point is inside the circle in local space
		/// @param point The point to test in local coordinates
		/// @return true if the point is inside or on the boundary
		[[nodiscard]] auto in(const Vec2& point) const noexcept -> bool;
	};
}
