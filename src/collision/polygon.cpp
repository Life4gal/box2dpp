// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/polygon.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/collision/hull.hpp>
#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto Polygon::compute_centroid(const std::span<const Vec2> vertices) noexcept -> Vec2
	{
		constexpr auto inv_3 = 1.f / 3.f;

		Vec2 center{.x = 0, .y = 0};
		float area = 0;

		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		const auto origin = vertices[0];
		for (std::size_t i = 1; i < vertices.size() - 1; ++i)
		{
			const auto& vertex1 = vertices[i];
			const auto& vertex2 = vertices[i + 1];

			// Triangle edges
			const auto edge1 = vertex1 - origin;
			const auto edge2 = vertex2 - origin;
			const auto a = .5f * edge1.cross(edge2);

			// Area weighted centroid
			center += a * inv_3 * (edge1 + edge2);
			area += a;
		}

		BPP_ASSERT(area > std::numeric_limits<float>::epsilon());
		const auto inv_area = 1.f / area;
		center *= inv_area;

		// Restore offset
		center += origin;

		return center;
	}

	auto Polygon::make_convex(const Hull& hull, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(hull.valid());

		// Handle a bad hull when assertions are disabled
		if (hull.count < 3)
		{
			return make_square(.5f);
		}

		Polygon result
		{
				.vertices = {},
				.normals = {},
				.centroid = {.x = 0, .y = 0},
				.radius = radius,
				.count = hull.count,
		};

		// Copy vertices
		std::ranges::copy(std::views::counted(hull.points, hull.count), result.vertices);

		// Compute normals. Ensure the edges have non-zero length.
		for (std::uint32_t index_v1 = 0; index_v1 < result.count; ++index_v1)
		{
			const auto index_v2 = (index_v1 + 1) % result.count;

			const auto& v1 = result.vertices[index_v1];
			const auto& v2 = result.vertices[index_v2];

			const auto edge = v2 - v1;
			BPP_ASSERT(edge.dot(edge) > std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon());
			result.normals[index_v1] = edge.cross(1.f).normalize();
		}

		result.centroid = compute_centroid({result.vertices, result.vertices + result.count});

		return result;
	}

	auto Polygon::make_offset_convex(const Hull& hull, const Vec2& position, const Rotation& rotation) noexcept -> Polygon
	{
		return make_offset_convex(hull, position, rotation, .0f);
	}

	auto Polygon::make_offset_convex(const Hull& hull, const Vec2& position, const Rotation& rotation, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(hull.valid());

		// Handle a bad hull when assertions are disabled
		if (hull.count < 3)
		{
			return make_square(.5f);
		}

		const Transform transform{.point = position, .rotation = rotation};

		Polygon result
		{
				.vertices = {},
				.normals = {},
				.centroid = {.x = 0, .y = 0},
				.radius = radius,
				.count = hull.count,
		};

		// Copy vertices
		std::ranges::transform(
			std::views::counted(hull.points, hull.count),
			result.vertices,
			[&transform](const Vec2& point) noexcept -> Vec2
			{
				return transform.transform(point);
			}
		);

		// Compute normals. Ensure the edges have non-zero length.
		for (std::uint32_t index_v1 = 0; index_v1 < result.count; ++index_v1)
		{
			const auto index_v2 = (index_v1 + 1) % result.count;

			const auto& v1 = result.vertices[index_v1];
			const auto& v2 = result.vertices[index_v2];

			const auto edge = v2 - v1;
			BPP_ASSERT(edge.dot(edge) > std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon());
			result.normals[index_v1] = edge.cross(1.f).normalize();
		}

		result.centroid = compute_centroid({result.vertices, result.vertices + result.count});

		return result;
	}

	auto Polygon::make_square(const float half_width) noexcept -> Polygon
	{
		return make_box(half_width, half_width);
	}

	auto Polygon::make_box(const float half_width, const float half_height) noexcept -> Polygon
	{
		return make_rounded_box(half_width, half_height, .0f);
	}

	auto Polygon::make_rounded_box(const float half_width, const float half_height, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(box2dpp::valid(half_width) and box2dpp::valid(half_height));
		BPP_ASSERT(half_width > 0 and half_height > 0);
		BPP_ASSERT(box2dpp::valid(radius) and radius >= 0);

		return
		{
				.vertices =
				{
						{.x = -half_width, .y = -half_height},
						{.x = half_width, .y = -half_height},
						{.x = half_width, .y = half_height},
						{.x = -half_width, .y = half_height},
				},
				.normals =
				{
						{.x = 0, .y = -1},
						{.x = 1, .y = 0},
						{.x = 0, .y = 1},
						{.x = -1, .y = 0},
				},
				.centroid = {.x = 0, .y = 0},
				.radius = radius,
				.count = 4,
		};
	}

	auto Polygon::make_offset_box(const float half_width, const float half_height, const Vec2& center, const Rotation& rotation) noexcept -> Polygon
	{
		return make_offset_rounded_box(half_width, half_height, center, rotation, .0f);
	}

	auto Polygon::make_offset_rounded_box(const float half_width, const float half_height, const Vec2& center, const Rotation& rotation, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(box2dpp::valid(radius) and radius > 0);

		const Transform transform{.point = center, .rotation = rotation};

		return
		{
				.vertices =
				{
						transform.transform({.x = -half_width, .y = -half_height}),
						transform.transform({.x = half_width, .y = -half_height}),
						transform.transform({.x = half_width, .y = half_height}),
						transform.transform({.x = -half_width, .y = half_height}),
				},
				.normals =
				{
						transform.rotation.rotate({.x = 0, .y = -1}),
						transform.rotation.rotate({.x = 1, .y = 0}),
						transform.rotation.rotate({.x = 0, .y = 1}),
						transform.rotation.rotate({.x = -1, .y = 0}),
				},
				.centroid = {.x = 0, .y = 0},
				.radius = radius,
				.count = 4,
		};
	}

	auto Polygon::transform(const Transform& transform, const Polygon& polygon) noexcept -> Polygon
	{
		auto p = polygon;

		for (auto [vertex, normal]: std::views::zip(
			     std::views::counted(p.vertices, p.count),
			     std::views::counted(p.normals, p.count)
		     ))
		{
			vertex = transform.transform(vertex);
			normal = transform.rotation.rotate(normal);
		}

		p.centroid = transform.transform(p.centroid);

		return p;
	}

	auto Polygon::in(const Vec2& point) const noexcept -> bool
	{
		const DistanceInput input
		{
				.proxy_a = ShapeProxy::create({vertices, count}, 0),
				.proxy_b = ShapeProxy::create({&point, 1}, 0),
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.use_radii = false
		};

		const auto output = DistanceOutput::compute(input, {});

		return output.distance <= radius;
	}
}
