// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A convex hull. Used to create convex polygons.
	class Hull final
	{
	public:
		/// The final points of the hull
		Vec2 points[BPP_MAX_POLYGON_VERTICES];

		/// The number of points
		std::uint32_t count;

	private:
		[[nodiscard]] static auto recurse_create(const Vec2& p1, const Vec2& p2, std::span<const Vec2> points) noexcept -> Hull;

	public:
		/// Compute the convex hull of a set of points. 
		/// Returns an empty hull if it fails.
		/// Some failure cases:
		/// - all points very close together
		/// - all points on a line
		/// - less than 3 points
		/// - more than BPP_MAX_POLYGON_VERTICES points
		/// This welds close points and removes collinear points.
		/// @warning Do not modify a hull once it has been computed
		[[nodiscard]] static auto create(std::span<const Vec2> points) noexcept -> Hull;

		/// This determines if a hull is valid. 
		/// Checks for:
		/// - convexity
		/// - collinear points
		/// This is expensive and should not be called at runtime.
		[[nodiscard]] auto valid() const noexcept -> bool;
	};
}
