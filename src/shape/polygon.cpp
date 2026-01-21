// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/polygon.hpp>

#include <algorithm>
#include <ranges>

#if BPP_VALIDATE
#include <print>
#endif

#include <box2dpp/utility/value_cast.hpp>

#include <box2dpp/shape/aabb.hpp>

// Polygon::in
#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto Hull::recurse_create(const Vec2& p1, const Vec2& p2, const std::span<const Vec2> points) noexcept -> Hull
	{
		Hull result{.points = {}, .count = 0};

		if (points.empty())
		{
			return result;
		}

		const auto linear_slop = BPP_LINEAR_SLOP;
		// create an edge vector pointing from p1 to p2
		const auto e = (p2 - p1).normalize();

		// discard points left of e and find point furthest to the right of e
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> right_points;
		std::ptrdiff_t right_count = 0;
		std::ptrdiff_t best_index = 0;
		auto best_distance = (points[best_index] - p1).cross(e);

		if (best_distance > 0)
		{
			right_points[right_count] = points[best_index];
			right_count += 1;
		}

		for (std::size_t index = 1; index < points.size(); ++index)
		{
			const auto distance = (points[index] - p1).cross(e);

			if (distance > best_distance)
			{
				best_index = value_cast<std::ptrdiff_t>(index);
				best_distance = distance;
			}

			if (distance > 0)
			{
				right_points[right_count] = points[index];
				right_count += 1;
			}
		}

		if (best_distance < linear_slop * 2.f)
		{
			return result;
		}

		const auto best_point = points[best_index];
		// compute hull to the right of p1-best_point
		const auto h1 = recurse_create(p1, best_point, {right_points.data(), static_cast<std::size_t>(right_count)});
		// compute hull to the right of best_point-p2
		const auto h2 = recurse_create(best_point, p2, {right_points.data(), static_cast<std::size_t>(right_count)});

		// stitch together hulls

		std::ranges::copy(std::views::counted(h1.points.data(), h1.count), result.points.data() + result.count);
		result.count += h1.count;

		result.points[result.count] = best_point;
		result.count += 1;

		std::ranges::copy(std::views::counted(h2.points.data(), h2.count), result.points.data() + result.count);
		result.count += h2.count;

		BPP_ASSERT(result.count < BPP_MAX_POLYGON_VERTICES);

		return result;
	}

	auto Hull::create(const std::span<const Vec2> points) noexcept -> Hull
	{
		Hull result{.points = {}, .count = 0};

		if (points.size() < 3 or points.size() > BPP_MAX_POLYGON_VERTICES)
		{
#if BPP_VALIDATE
			std::println(stderr, "[Hull] Too few or too many points to create hull: {}", points.size());
#endif

			return result;
		}

		const auto linear_slop = BPP_LINEAR_SLOP;
		const auto tolerance_2 = linear_slop * linear_slop * 16.f;

		AABB aabb
		{
				.lower = {.x = std::numeric_limits<float>::max(), .y = std::numeric_limits<float>::max()},
				.upper = {.x = std::numeric_limits<float>::lowest(), .y = std::numeric_limits<float>::lowest()},
		};

		// Perform aggressive point welding. First point always remains.
		// Also compute the bounding box for later.
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> computed_points;
		std::ptrdiff_t computed_count = 0;
		for (std::size_t index = 0; index < points.size(); ++index)
		{
			const auto& point = points[index];

			aabb = aabb.combination_max(point);

			const auto unique = std::ranges::none_of(
				std::ranges::subrange{points.begin(), points.begin() + value_cast<std::ptrdiff_t>(index)},
				[&point, tolerance_2](const Vec2& prev_point) noexcept -> bool // NOLINT(clang-diagnostic-padded)
				{
					const auto distance_2 = point.distance_squared(prev_point);
					return distance_2 < tolerance_2;
				}
			);

			if (unique)
			{
				computed_points[computed_count] = point;
				computed_count += 1;
			}
		}

		if (computed_count < 3)
		{
#if BPP_VALIDATE
			std::println(stderr, "[Hull] All points very close together, check your data and check your scale");
#endif
			return result;
		}

		const auto find_furthest_point_index = [computed_points](const std::ptrdiff_t c, const Vec2 base) noexcept -> std::ptrdiff_t
		{
			const auto r = std::views::counted(computed_points.data(), c);
			const auto it = std::ranges::max_element(
				r,
				{},
				[base](const Vec2& p) noexcept -> float
				{
					return base.distance_squared(p);
				}
			);

			return std::ranges::distance(r.begin(), it);
		};

		// Find an extreme point as the first point on the hull
		const auto center = aabb.center();
		const auto index_p1 = find_furthest_point_index(computed_count, center);
		const auto p1 = computed_points[index_p1];

		// remove p1 from working set
		computed_points[index_p1] = computed_points[computed_count - 1];
		computed_count -= 1;

		const auto index_p2 = find_furthest_point_index(computed_count, p1);
		const auto p2 = computed_points[index_p2];

		// remove p2 from working set
		computed_points[index_p2] = computed_points[computed_count - 1];
		computed_count -= 1;

		const auto e = (p2 - p1).normalize();

		// split the points into points that are left and right of the line p1-p2.
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES - 2> right_points;
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES - 2> left_points;
		std::ptrdiff_t right_count = 0;
		std::ptrdiff_t left_count = 0;

		std::ranges::for_each(
			std::views::counted(computed_points.data(), computed_count),
			[&](const Vec2& p) noexcept -> void
			{
				// slop used here to skip points that are very close to the line p1-p2
				if (const auto d = (p - p1).cross(e);
					d >= linear_slop * 2.f)
				{
					right_points[right_count] = p;
					right_count += 1;
				}
				else if (d <= -linear_slop * 2.f)
				{
					left_points[left_count] = p;
					left_count += 1;
				}
			}
		);

		// compute hulls on right and left
		const auto h1 = recurse_create(p1, p2, {right_points.data(), static_cast<std::size_t>(right_count)});
		const auto h2 = recurse_create(p2, p1, {left_points.data(), static_cast<std::size_t>(left_count)});

		if (h1.count == 0 or h2.count == 0)
		{
#if BPP_VALIDATE
			std::println(stderr, "[Hull] All points collinear");
#endif
			return result;
		}

		// stitch hulls together, preserving CCW winding order

		result.points[result.count] = p1;
		result.count += 1;

		std::ranges::copy(std::views::counted(h1.points.data(), h1.count), result.points.data() + result.count);
		result.count += h1.count;

		result.points[result.count] = p2;
		result.count += 1;

		std::ranges::copy(std::views::counted(h2.points.data(), h2.count), result.points.data() + result.count);
		result.count += h2.count;

		BPP_ASSERT(result.count <= BPP_MAX_POLYGON_VERTICES);

		// merge collinear
		for (bool searching = true; searching and result.count > 2;)
		{
			searching = false;

			for (std::uint32_t i1 = 0; i1 < result.count; ++i1)
			{
				const auto i2 = (i1 + 1) % result.count;
				const auto i3 = (i1 + 2) % result.count;

				const auto& s1 = result.points[i1];
				const auto& s2 = result.points[i2];
				const auto& s3 = result.points[i3];

				// unit edge vector for s1-s3
				const auto r = (s3 - s1).normalize();

				if (const auto distance = (s2 - s1).cross(r);
					distance <= linear_slop * 2.f)
				{
					// remove midpoint from hull
					for (std::uint32_t j = i2; j < result.count - 1; ++j)
					{
						result.points[j] = result.points[j + 1];
					}
					result.count -= 1;

					// continue searching for collinear points
					searching = true;

					break;
				}
			}
		}

		if (result.count < 3)
		{
#if BPP_VALIDATE
			std::println(stderr, "[Hull] All points collinear, shouldn't be reached since this was validated above");
#endif
			result.count = 0;
		}

		return result;
	}

	auto Hull::valid() const noexcept -> bool
	{
		if (count < 3 or count > BPP_MAX_POLYGON_VERTICES)
		{
			return false;
		}

		// test that every point is behind every edge
		for (std::size_t index_p1 = 0; index_p1 < count; ++index_p1)
		{
			const auto index_p2 = index_p1 < count - 1 ? index_p1 + 1 : 0;

			const auto& p1 = points[index_p1];
			const auto& p2 = points[index_p2];

			const auto e = (p2 - p1).normalize();

			for (std::size_t index = 0; index < count; ++index)
			{
				// skip points that subtend the current edge
				if (index == index_p1 or index == index_p2)
				{
					continue;
				}

				if (const auto distance = (points[index] - p1).cross(e);
					distance >= 0)
				{
					return false;
				}
			}
		}

		// test for collinear points
		const auto linear_slop = BPP_LINEAR_SLOP;
		for (std::size_t index_p1 = 0; index_p1 < count; ++index_p1)
		{
			const auto index_p2 = (index_p1 + 1) % count;
			const auto index_p3 = (index_p1 + 2) % count;

			const auto& p1 = points[index_p1];
			const auto& p2 = points[index_p2];
			const auto& p3 = points[index_p3];

			const auto e = (p3 - p1).normalize();

			if (const auto distance = (p2 - p1).cross(e);
				distance <= linear_slop)
			{
				// p1-p2-p3 are collinear
				return false;
			}
		}

		return true;
	}

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

	auto Polygon::make(const Hull& hull, const float radius) noexcept -> Polygon
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
		std::ranges::copy(std::views::counted(hull.points.data(), hull.count), result.vertices.data());

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

		result.centroid = compute_centroid({result.vertices.data(), result.count});

		return result;
	}

	auto Polygon::make(const Hull& hull, const Vec2& position, const Rotation& rotation) noexcept -> Polygon
	{
		return make(hull, position, rotation, .0f);
	}

	auto Polygon::make(const Hull& hull, const Vec2& position, const Rotation& rotation, const float radius) noexcept -> Polygon
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
			std::views::counted(hull.points.data(), hull.count),
			result.vertices.data(),
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

		result.centroid = compute_centroid({result.vertices.data(), result.count});

		return result;
	}

	auto Polygon::make_square(const float half_width) noexcept -> Polygon
	{
		return make_box(half_width, half_width);
	}

	auto Polygon::make_box(const float half_width, const float half_height) noexcept -> Polygon
	{
		return make_box(half_width, half_height, .0f);
	}

	auto Polygon::make_box(const float half_width, const float half_height, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(box2dpp::valid(half_width) and box2dpp::valid(half_height));
		BPP_ASSERT(half_width > 0 and half_height > 0);
		BPP_ASSERT(box2dpp::valid(radius) and radius >= 0);

		return
		{
				.vertices =
				{{
						{.x = -half_width, .y = -half_height},
						{.x = half_width, .y = -half_height},
						{.x = half_width, .y = half_height},
						{.x = -half_width, .y = half_height},
				}},
				.normals =
				{{
						{.x = 0, .y = -1},
						{.x = 1, .y = 0},
						{.x = 0, .y = 1},
						{.x = -1, .y = 0},
				}},
				.centroid = {.x = 0, .y = 0},
				.radius = radius,
				.count = 4,
		};
	}

	auto Polygon::make_box(const float half_width, const float half_height, const Vec2& center, const Rotation& rotation) noexcept -> Polygon
	{
		return make_box(half_width, half_height, center, rotation, .0f);
	}

	auto Polygon::make_box(const float half_width, const float half_height, const Vec2& center, const Rotation& rotation, const float radius) noexcept -> Polygon
	{
		BPP_ASSERT(box2dpp::valid(radius) and radius >= 0);

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
			     std::views::counted(p.vertices.data(), p.count),
			     std::views::counted(p.normals.data(), p.count)
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
				.proxy_a = ShapeProxy::create({vertices.data(), count}, 0),
				.proxy_b = ShapeProxy::create({&point, 1}, 0),
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.use_radii = false
		};

		const auto output = Distance::compute(input);

		return output.distance <= radius;
	}
}
