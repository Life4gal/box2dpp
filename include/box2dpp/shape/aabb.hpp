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
		/// Minimum point (bottom-left) of the AABB
		Vec2 lower;
		/// Maximum point (top-right) of the AABB
		Vec2 upper;

		[[nodiscard]] static auto compute(std::span<const Vec2> points, float radius) noexcept -> AABB;

		/// Compute the bounding box of a transformed circle
		[[nodiscard]] static auto compute(const Circle& circle, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed capsule
		[[nodiscard]] static auto compute(const Capsule& capsule, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed polygon
		[[nodiscard]] static auto compute(const Polygon& polygon, const Transform& transform) noexcept -> AABB;

		/// Compute the bounding box of a transformed line segment
		[[nodiscard]] static auto compute(const Segment& segment, const Transform& transform) noexcept -> AABB;

		/// Check if the AABB is valid (lower <= upper and both points are valid)
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Check if this AABB fully contains another AABB
		[[nodiscard]] auto contains(const AABB& other) const noexcept -> bool;

		/// Check if a point is inside the AABB
		[[nodiscard]] auto contains(const Vec2& point) const noexcept -> bool;

		/// Check if this AABB overlaps with another AABB
		[[nodiscard]] auto overlaps(const AABB& other) const noexcept -> bool;

		/// Get the center point of the AABB
		[[nodiscard]] auto center() const noexcept -> Vec2;

		/// Get the extents (half-width and half-height) of the AABB
		[[nodiscard]] auto extents() const noexcept -> Vec2;

		/// Get the perimeter (surface area) of the AABB
		[[nodiscard]] auto perimeter() const noexcept -> float;

		/// Get the width of the AABB
		[[nodiscard]] auto width() const noexcept -> float;

		/// Get the height of the AABB
		[[nodiscard]] auto height() const noexcept -> float;

		/// Get the area of the AABB
		[[nodiscard]] auto area() const noexcept -> float;

		/// Component-wise combination using a binary functor
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

		/// Component-wise combination using a compile-time binary function
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

		/// Component-wise combination using a member function
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

		/// Combine with another AABB by taking the minimum bounds (intersection)
		[[nodiscard]] auto combination_min(const AABB& other) const noexcept -> AABB;

		/// Combine with another AABB by taking the maximum bounds (union)
		[[nodiscard]] auto combination_max(const AABB& other) const noexcept -> AABB;

		/// Component-wise combination using a compile-time binary function
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

		/// Component-wise combination using a member function
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

		/// Combine with a point by taking the minimum bounds
		[[nodiscard]] auto combination_min(const Vec2& point) const noexcept -> AABB;

		/// Combine with a point by taking the maximum bounds
		[[nodiscard]] auto combination_max(const Vec2& point) const noexcept -> AABB;

		/// Enlarge this AABB to contain another AABB
		/// @return true if the AABB was actually enlarged
		auto enlarge(const AABB& other) noexcept -> bool;

		/// Enlarge this AABB to contain a point
		/// @return true if the AABB was actually enlarged
		auto enlarge(const Vec2& point) noexcept -> bool;
	};
}
