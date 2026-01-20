// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math/vec2.hpp>
#include <box2dpp/math/transform.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	/// Axis-aligned bounding box
	// ReSharper disable once CppInconsistentNaming
	class AABB final
	{
	public:
		Vec2 lower;
		Vec2 upper;

		/// Compute the bounding box of an array of circles
		[[nodiscard]] static auto compute(std::span<const Vec2> points, float radius) noexcept -> AABB;

		/// Compute the bounding box of a transformed circle
		[[nodiscard]] static auto compute(const Circle& circle, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed capsule
		[[nodiscard]] static auto compute(const Capsule& capsule, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed polygon
		[[nodiscard]] static auto compute(const Polygon& polygon, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed line segment
		[[nodiscard]] static auto compute(const Segment& segment, const Transform& transform) noexcept -> AABB;

		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Does this box fully contain another one
		[[nodiscard]] auto contains(const AABB& other) const noexcept -> bool;

		/// Does this box overlapped another one
		[[nodiscard]] auto overlaps(const AABB& other) const noexcept -> bool;

		/// Get the center of the AABB.
		[[nodiscard]] auto center() const noexcept -> Vec2;

		/// Get the extents of the AABB (half-widths).
		[[nodiscard]] auto extents() const noexcept -> Vec2;

		// Get surface area of an AABB (the perimeter length)
		[[nodiscard]] auto perimeter() const noexcept -> float;

		template<typename LowerFunctor, typename UpperFunctor>
		[[nodiscard]] constexpr auto combination(
			const AABB& other,
			LowerFunctor lower_functor,
			UpperFunctor upper_functor
		) const noexcept -> AABB //
			requires requires
			{
				lower_functor(lower, other.lower);
				upper_functor(upper, other.upper);
			}
		{
			return {.lower = lower_functor(lower, other.lower), .upper = upper_functor(upper, other.upper)};
		}

		template<auto LowerFunctor, auto UpperFunctor>
		[[nodiscard]] constexpr auto combination(const AABB& other) const noexcept -> AABB //
			requires requires
			{
				LowerFunctor(lower, other.lower);
				UpperFunctor(upper, other.upper);
			}
		{
			return {.lower = LowerFunctor(lower, other.lower), .upper = UpperFunctor(upper, other.upper)};
		}

		template<auto LowerFunctor, auto UpperFunctor>
		[[nodiscard]] constexpr auto combination(const AABB& other) const noexcept -> AABB //
			requires requires
			{
				(lower.*LowerFunctor)(other.lower);
				(upper.*UpperFunctor)(other.upper);
			}
		{
			return {.lower = (lower.*LowerFunctor)(other.lower), .upper = (upper.*UpperFunctor)(other.upper)};
		}

		[[nodiscard]] auto combination_min(const AABB& other) const noexcept -> AABB;

		[[nodiscard]] auto combination_max(const AABB& other) const noexcept -> AABB;

		template<auto LowerFunctor, auto UpperFunctor>
		[[nodiscard]] constexpr auto combination(const Vec2& point) const noexcept -> AABB //
			requires requires
			{
				LowerFunctor(lower, point);
				UpperFunctor(upper, point);
			}
		{
			return {.lower = LowerFunctor(lower, point), .upper = UpperFunctor(upper, point)};
		}

		template<auto LowerFunctor, auto UpperFunctor>
		[[nodiscard]] constexpr auto combination(const Vec2& point) const noexcept -> AABB //
			requires requires
			{
				(lower.*LowerFunctor)(point);
				(upper.*UpperFunctor)(point);
			}
		{
			return {.lower = (lower.*LowerFunctor)(point), .upper = (upper.*UpperFunctor)(point)};
		}

		[[nodiscard]] auto combination_min(const Vec2& point) const noexcept -> AABB;

		[[nodiscard]] auto combination_max(const Vec2& point) const noexcept -> AABB;

		/// Enlarge this box to contain another one
		/// @return true if the AABB grew
		auto enlarge(const AABB& other) noexcept -> bool;
	};
}
