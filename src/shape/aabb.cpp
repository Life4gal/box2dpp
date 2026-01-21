// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/aabb.hpp>

#include <algorithm>
#include <functional>
#include <ranges>

#include <box2dpp/shape/circle.hpp>
#include <box2dpp/shape/capsule.hpp>
#include <box2dpp/shape/polygon.hpp>
#include <box2dpp/shape/segment.hpp>

namespace box2dpp
{
	auto AABB::compute(std::span<const Vec2> points, float radius) noexcept -> AABB
	{
		// todo
		return {};
	}

	auto AABB::compute(const Circle& circle, const Transform& transform) noexcept -> AABB
	{
		// Transform circle center to world space
		const auto p = transform.transform(circle.center);

		// Expand by radius in all directions
		return {.lower = p - circle.radius, .upper = p + circle.radius};
	}

	auto AABB::compute(const Capsule& capsule, const Transform& transform) noexcept -> AABB
	{
		// Transform both endpoints to world space
		const auto p1 = transform.transform(capsule.center1);
		const auto p2 = transform.transform(capsule.center2);

		// Find min and max of transformed endpoints
		const auto min_p = p1.combination_min(p2);
		const auto max_p = p1.combination_max(p2);

		return {.lower = min_p - capsule.radius, .upper = max_p + capsule.radius};
	}

	auto AABB::compute(const Polygon& polygon, const Transform& transform) noexcept -> AABB
	{
		BPP_ASSERT(polygon.count > 0);

		// Initialize with extreme values
		Vec2 min_p{.x = std::numeric_limits<float>::max(), .y = std::numeric_limits<float>::max()};
		Vec2 max_p{.x = std::numeric_limits<float>::min(), .y = std::numeric_limits<float>::min()};

		// Transform all vertices and find bounds
		std::ranges::for_each(
			std::views::counted(polygon.vertices.data(), polygon.count),
			[&min_p, &max_p, &transform](const Vec2& vertex) noexcept -> void
			{
				const auto v = transform.transform(vertex);

				min_p = min_p.combination_min(v);
				max_p = max_p.combination_max(v);
			}
		);

		return {.lower = min_p - polygon.radius, .upper = max_p + polygon.radius};
	}

	auto AABB::compute(const Segment& segment, const Transform& transform) noexcept -> AABB
	{
		// Transform segment endpoints
		const auto p1 = transform.transform(segment.point1);
		const auto p2 = transform.transform(segment.point2);

		// Find bounds of transformed endpoints (segments have zero radius)
		const auto min_p = p1.combination_min(p2);
		const auto max_p = p1.combination_max(p2);

		return {.lower = min_p, .upper = max_p};
	}

	auto AABB::valid() const noexcept -> bool
	{
		// Check if lower bounds are less than or equal to upper bounds
		if (const auto [x, y] = upper - lower;
			x < 0.f or y < 0.f)
		{
			return false;
		}

		return lower.valid() and upper.valid();
	}

	auto AABB::contains(const AABB& other) const noexcept -> bool
	{
		// This AABB contains another if it completely encloses it
		return
				lower.x <= other.lower.x and
				lower.y <= other.lower.y and
				upper.x >= other.upper.x and
				upper.y >= other.upper.y;
	}

	auto AABB::contains(const Vec2& point) const noexcept -> bool
	{
		// Check if point is within AABB bounds (inclusive)
		return
				lower.x <= point.x and
				lower.y <= point.y and
				upper.x >= point.x and
				upper.y >= point.y;
	}

	auto AABB::overlaps(const AABB& other) const noexcept -> bool
	{
		// Check for separating axis on X
		if (other.lower.x > upper.x or other.upper.x < lower.x)
		{
			return false;
		}

		// Check for separating axis on Y
		if (other.lower.y > upper.y or other.upper.y < lower.y)
		{
			return false;
		}

		return true;
	}

	auto AABB::center() const noexcept -> Vec2
	{
		// Midpoint between lower and upper bounds
		return (lower + upper) / 2.f;
	}

	auto AABB::extents() const noexcept -> Vec2
	{
		// Half of the dimensions
		return (upper - lower) / 2.f;
	}

	auto AABB::perimeter() const noexcept -> float
	{
		// Perimeter = 2 * (width + height)
		return (upper.x - lower.x + upper.y - lower.y) * 2.f;
	}

	auto AABB::width() const noexcept -> float
	{
		return upper.x - lower.x;
	}

	auto AABB::height() const noexcept -> float
	{
		return upper.y - lower.y;
	}

	auto AABB::area() const noexcept -> float
	{
		return width() * height();
	}

	auto AABB::combination_min(const AABB& other) const noexcept -> AABB
	{
		// Intersection: take maximum of lower bounds, minimum of upper bounds
		return combination<&Vec2::combination_max, &Vec2::combination_min>(other);
	}

	auto AABB::combination_max(const AABB& other) const noexcept -> AABB
	{
		// Union: take minimum of lower bounds, maximum of upper bounds
		return combination<&Vec2::combination_min, &Vec2::combination_max>(other);
	}

	auto AABB::combination_min(const Vec2& point) const noexcept -> AABB
	{
		// Intersection with point (degenerate case)
		return combination<&Vec2::combination_max, &Vec2::combination_min>(point);
	}

	auto AABB::combination_max(const Vec2& point) const noexcept -> AABB
	{
		// Union with point (expand to include point)
		return combination<&Vec2::combination_min, &Vec2::combination_max>(point);
	}

	auto AABB::enlarge(const AABB& other) noexcept -> bool
	{
		// Compute union with other AABB
		const auto new_one = combination_max(other);

		// Check if actually changed
		if (lower == new_one.lower and upper == new_one.upper)
		{
			return false;
		}

		*this = new_one;
		return true;
	}

	auto AABB::enlarge(const Vec2& point) noexcept -> bool
	{
		// Enlarge to include point
		return enlarge(AABB{.lower = point, .upper = point});
	}
}
