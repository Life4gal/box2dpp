// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/aabb.hpp>

#include <algorithm>
#include <functional>
#include <ranges>

#include <box2dpp/collision/circle.hpp>
#include <box2dpp/collision/capsule.hpp>
#include <box2dpp/collision/polygon.hpp>
#include <box2dpp/collision/segment.hpp>

namespace box2dpp
{
	auto AABB::valid() const noexcept -> bool
	{
		if (const auto [x, y] = upper - lower; x < 0.f or y < 0.f)
		{
			return false;
		}

		return lower.valid() and upper.valid();
	}

	auto AABB::contains(const AABB& other) const noexcept -> bool
	{
		return
				lower.x <= other.lower.x and
				lower.y <= other.lower.y and
				upper.x >= other.upper.x and
				upper.y >= other.upper.y;
	}

	auto AABB::overlaps(const AABB& other) const noexcept -> bool
	{
		if (other.lower.x > upper.x or other.upper.x < lower.x)
		{
			return false;
		}

		if (other.lower.y > upper.y or other.upper.y < lower.y)
		{
			return false;
		}

		return true;
	}

	auto AABB::center() const noexcept -> Vec2
	{
		return (lower + upper) / 2.f;
	}

	auto AABB::extents() const noexcept -> Vec2
	{
		return (upper - lower) / 2.f;
	}

	auto AABB::perimeter() const noexcept -> float
	{
		return (upper.x - lower.x + upper.y - lower.y) * 2.f;
	}

	auto AABB::combination_min(const AABB& other) const noexcept -> AABB
	{
		return combination<&Vec2::combination_max, &Vec2::combination_min>(other);
	}

	auto AABB::combination_max(const AABB& other) const noexcept -> AABB
	{
		return combination<&Vec2::combination_min, &Vec2::combination_max>(other);
	}

	auto AABB::combination_min(const Vec2& point) const noexcept -> AABB
	{
		return combination<&Vec2::combination_max, &Vec2::combination_min>(point);
	}

	auto AABB::combination_max(const Vec2& point) const noexcept -> AABB
	{
		return combination<&Vec2::combination_min, &Vec2::combination_max>(point);
	}

	auto AABB::enlarge(const AABB& other) noexcept -> bool
	{
		const auto new_one = combination_max(other);

		if (lower == new_one.lower and upper == new_one.upper)
		{
			return false;
		}

		*this = new_one;
		return true;
	}

	auto AABB::compute(std::span<const Vec2> points, float radius) noexcept -> AABB
	{
		// todo
		return {};
	}

	auto AABB::compute(const Circle& circle, const Transform& transform) noexcept -> AABB
	{
		const auto p = transform.transform(circle.center);

		return {.lower = p - circle.radius, .upper = p + circle.radius};
	}

	auto AABB::compute(const Capsule& capsule, const Transform& transform) noexcept -> AABB
	{
		const auto p1 = transform.transform(capsule.center1);
		const auto p2 = transform.transform(capsule.center2);

		const auto min_p = p1.combination(p2, std::ranges::min);
		const auto max_p = p1.combination(p2, std::ranges::max);

		return {.lower = min_p - capsule.radius, .upper = max_p + capsule.radius};
	}

	auto AABB::compute(const Polygon& polygon, const Transform& transform) noexcept -> AABB
	{
		BPP_ASSERT(polygon.count > 0);

		Vec2 min_p{.x = std::numeric_limits<float>::max(), .y = std::numeric_limits<float>::max()};
		Vec2 max_p{.x = std::numeric_limits<float>::min(), .y = std::numeric_limits<float>::min()};

		std::ranges::for_each(
			std::views::counted(polygon.vertices, polygon.count),
			[&min_p, &max_p, &transform](const Vec2& vertex) noexcept -> void
			{
				const auto v = transform.transform(vertex);

				min_p = min_p.combination(v, std::ranges::min);
				max_p = max_p.combination(v, std::ranges::max);
			}
		);

		return {.lower = min_p - polygon.radius, .upper = max_p + polygon.radius};
	}

	auto AABB::compute(const Segment& segment, const Transform& transform) noexcept -> AABB
	{
		const auto p1 = transform.transform(segment.point1);
		const auto p2 = transform.transform(segment.point2);

		const auto min_p = p1.combination(p2, std::ranges::min);
		const auto max_p = p1.combination(p2, std::ranges::max);

		return {.lower = min_p, .upper = max_p};
	}
}
