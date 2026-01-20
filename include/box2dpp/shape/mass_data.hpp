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

	/// This holds the mass data computed for a shape.
	class MassData final
	{
	public:
		/// The mass of the shape, usually in kilograms.
		float mass;

		/// The position of the shape's centroid relative to the shape's origin.
		Vec2 center;

		/// The rotational inertia of the shape about the shape center.
		float rotational_inertia;

		/// Compute mass properties of a circle
		[[nodiscard]] static auto compute(const Circle& circle, float density) noexcept -> MassData;

		/// Compute mass properties of a capsule
		[[nodiscard]] static auto compute(const Capsule& capsule, float density) noexcept -> MassData;

		/// Compute mass properties of a polygon
		[[nodiscard]] static auto compute(const Polygon& polygon, float density) noexcept -> MassData;
	};
}
