// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/collision/simplex.hpp>

namespace box2dpp
{
	class DistanceInput final
	{
	public:
		/// The proxy for shape A
		ShapeProxy proxy_a;

		/// The proxy for shape B
		ShapeProxy proxy_b;

		/// The world transform for shape A
		Transform transform_a;

		/// The world transform for shape B
		Transform transform_b;

		/// Should the proxy radius be considered?
		bool use_radii;
	};

	class Distance final
	{
	public:
		/// Closest point on shape A in world coordinates
		Vec2 point_a;

		/// Closest point on shape B in world coordinates  
		Vec2 point_b;

		/// Unit normal pointing from A to B.
		/// Only valid when separated (distance > 0).
		Vec2 normal;

		/// Signed distance between shapes.
		/// Positive when separated, zero when touching, negative when overlapping.
		float distance;

		/// Compute the closest points between two shapes represented as point clouds.
		/// SimplexCache cache is input/output. On the first call set SimplexCache::count to zero.
		[[nodiscard]] static auto compute(const DistanceInput& input, SimplexCache& inout_cache) noexcept -> Distance;

		/// Compute the closest points between two shapes represented as point clouds.
		[[nodiscard]] static auto compute(const DistanceInput& input) noexcept -> Distance;
	};
}
