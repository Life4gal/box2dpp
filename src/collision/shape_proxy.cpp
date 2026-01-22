// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/shape_proxy.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/utility/value_cast.hpp>

#include <box2dpp/shape/circle.hpp>
#include <box2dpp/shape/capsule.hpp>
#include <box2dpp/shape/polygon.hpp>
#include <box2dpp/shape/segment.hpp>

namespace box2dpp
{
	auto ShapeProxy::from(const std::span<const Vec2> points, const float radius) noexcept -> ShapeProxy
	{
		BPP_ASSERT(not points.empty());
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = value_cast<std::uint32_t>(points.size()), .radius = radius};
		std::ranges::copy(points, result.points.data());

		return result;
	}

	auto ShapeProxy::from(const Circle& circle) noexcept -> ShapeProxy
	{
		return from({&circle.center, 1}, circle.radius);
	}

	auto ShapeProxy::from(const Capsule& capsule) noexcept -> ShapeProxy
	{
		// return create({&capsule.center1, 2}, capsule.radius);
		const Vec2 points[]{capsule.center1, capsule.center2};
		return from({points, 2}, capsule.radius);
	}

	auto ShapeProxy::from(const Polygon& polygon) noexcept -> ShapeProxy
	{
		return from({polygon.vertices.data(), static_cast<std::size_t>(polygon.count)}, polygon.radius);
	}

	auto ShapeProxy::from(const Segment& segment) noexcept -> ShapeProxy
	{
		// return create({&segment.point1, 2}, 0);
		const Vec2 points[]{segment.point1, segment.point2};
		return from({points, 2}, 0);
	}

	auto ShapeProxy::from(const std::span<const Vec2> points, const float radius, const Transform& transform) noexcept -> ShapeProxy
	{
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = value_cast<std::uint32_t>(points.size()), .radius = radius};
		std::ranges::transform(
			points,
			result.points.data(),
			[&transform](const Vec2& point) noexcept -> Vec2
			{
				return transform.transform(point);
			}
		);

		return result;
	}

	auto ShapeProxy::from(const Circle& circle, const Transform& transform) noexcept -> ShapeProxy
	{
		return from({&circle.center, 1}, circle.radius, transform);
	}

	auto ShapeProxy::from(const Capsule& capsule, const Transform& transform) noexcept -> ShapeProxy
	{
		// return create({&capsule.center1, 2}, capsule.radius);
		const Vec2 points[]{capsule.center1, capsule.center2};
		return from({points, 2}, capsule.radius, transform);
	}

	auto ShapeProxy::from(const Polygon& polygon, const Transform& transform) noexcept -> ShapeProxy
	{
		return from({polygon.vertices.data(), static_cast<std::size_t>(polygon.count)}, polygon.radius, transform);
	}

	auto ShapeProxy::from(const Segment& segment, const Transform& transform) noexcept -> ShapeProxy
	{
		// return create({&segment.point1, 2}, 0);
		const Vec2 points[]{segment.point1, segment.point2};
		return from({points, 2}, 0, transform);
	}

	auto ShapeProxy::valid() const noexcept -> bool
	{
		if (count == 0 or count > BPP_MAX_POLYGON_VERTICES)
		{
			return false;
		}

		if (radius < 0.0f or not box2dpp::valid(radius))
		{
			return false;
		}

		if (not std::ranges::all_of(
			std::views::counted(points.data(), count),
			&Vec2::valid
		))
		{
			return false;
		}

		return true;
	}

	auto ShapeProxy::find_support(const Vec2& direction) const noexcept -> index_type
	{
		BPP_ASSERT(count != 0);

		// Handle near-zero direction vector
		if (direction.length_squared() < std::numeric_limits<float>::epsilon())
		{
			// Return first point as default
			return 0;
		}

		std::ptrdiff_t best_index = 0;
		auto best_value = points[0].dot(direction);
		for (std::uint32_t i = 1; i < count; ++i)
		{
			if (const auto value = points[i].dot(direction);
				value > best_value)
			{
				best_index = i;
				best_value = value;
			}
		}

		return value_cast<index_type>(best_index);
	}
}
