// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <optional>

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	/// Input parameters for a ray cast operation
	/// Represents a ray segment from origin to origin + max_fraction * translation
	class RayCastInput final
	{
	public:
		/// Start point of the ray in local shape space
		Vec2 origin;

		/// Direction and magnitude of the ray (not necessarily normalized)
		Vec2 translation;

		/// Maximum fraction of translation to consider (0 to 1)
		/// A value of 1 mean consider the full translation vector
		float max_fraction;

		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// Result of a ray cast operation against a shape
	/// Contains intersection information when a hit occurs
	class RayCastOutput final
	{
	public:
		/// Surface normal at intersection point (points away from shape interior)
		/// Zero vector for initial overlaps
		Vec2 normal;

		/// World-space hit point on shape surface
		Vec2 point;

		/// Fraction of translation at which intersection occurred (0 to max_fraction)
		/// 0 indicates initial overlap at ray origin
		float fraction;
	};

	class RayCast final
	{
	public:
		/// Test ray against circle shape in local space
		/// @param input Ray parameters
		/// @param circle Circle shape in local coordinates
		/// @return RayCast result with hit information
		[[nodiscard]] static auto test(const RayCastInput& input, const Circle& circle) noexcept -> std::optional<RayCastOutput>;

		/// Test ray against capsule shape in local space
		/// @param input Ray parameters
		/// @param capsule Capsule shape in local coordinates
		/// @return RayCast result with hit information
		[[nodiscard]] static auto test(const RayCastInput& input, const Capsule& capsule) noexcept -> std::optional<RayCastOutput>;

		/// Test ray against convex polygon shape in local space
		/// Supports both sharp and rounded polygons
		/// @param input Ray parameters
		/// @param polygon Convex polygon in local coordinates
		/// @return RayCast result with hit information
		[[nodiscard]] static auto test(const RayCastInput& input, const Polygon& polygon) noexcept -> std::optional<RayCastOutput>;

		/// Test ray against line segment in local space
		/// @param input Ray parameters
		/// @param segment Line segment in local coordinates
		/// @param one_sided If true, only hits from the right side are considered
		///                  (useful for chain/edge shapes with one-sided collision)
		/// @return RayCast result with hit information
		[[nodiscard]] static auto test(const RayCastInput& input, const Segment& segment, bool one_sided) noexcept -> std::optional<RayCastOutput>;
	};
}
