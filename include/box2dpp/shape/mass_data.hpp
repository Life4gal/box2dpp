// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;

	/// Holds the mass data computed for a shape (mass, center of mass, and rotational inertia)
	class MassData final
	{
	public:
		/// Mass of the shape (usually in kilograms)
		float mass;

		/// Position of the shape's centroid relative to the shape's origin
		Vec2 center;

		/// Rotational inertia about the centroid
		float rotational_inertia;

		/// Compute mass properties of a circle
		[[nodiscard]] static auto compute(const Circle& circle, float density) noexcept -> MassData;

		/// Compute mass properties of a capsule
		[[nodiscard]] static auto compute(const Capsule& capsule, float density) noexcept -> MassData;

		/// Compute mass properties of a polygon
		[[nodiscard]] static auto compute(const Polygon& polygon, float density) noexcept -> MassData;

		/// Check if mass data is valid (positive mass and inertia, finite values)
		[[nodiscard]] auto valid() const noexcept -> bool;
	};
}
