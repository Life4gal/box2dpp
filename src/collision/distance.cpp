// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/distance.hpp>

#include <algorithm>
#include <ranges>

namespace box2dpp
{
	auto Distance::compute(const DistanceInput& input, SimplexCache& inout_cache) noexcept -> Distance
	{
		BPP_ASSERT(input.proxy_a.valid() and input.proxy_b.valid());

		// Get proxy B in frame A to avoid further transforms in the main loop.
		// This is still a performance gain at 8 points.
		ShapeProxy local_proxy_b{.points = {}, .count = input.proxy_b.count, .radius = input.proxy_b.radius};
		{
			const auto transform = input.transform_a.inv_multiply(input.transform_b);

			std::ranges::transform(
				std::views::counted(input.proxy_b.points.data(), input.proxy_b.count),
				local_proxy_b.points.data(),
				[&transform](const Vec2& point) noexcept -> Vec2
				{
					return transform.transform(point);
				}
			);
		}

		// Initialize the simplex
		auto simplex = Simplex::create(inout_cache, input.proxy_a, local_proxy_b);

		auto non_unit_normal = Vec2::zero;

		// Main iteration loop. 
		// All computations are done in frame A.
		{
			// These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
			SimplexVertex::index_type save_a[3];
			SimplexVertex::index_type save_b[3];

			for (std::uint32_t iteration = 0; iteration < BPP_COLLISION_DISTANCE_MAX_ITERATIONS; ++iteration)
			{
				// Copy simplex so we can identify duplicates.
				for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(simplex.type); ++i)
				{
					const auto& vertex = simplex.vertices[i];

					save_a[i] = vertex.index_a;
					save_b[i] = vertex.index_b;
				}

				const auto d = simplex.solve();

				const auto fallback_overlap_result = [&] noexcept -> Distance
				{
					// Overlap
					const auto [local_point_a, local_point_b] = simplex.compute_closest_points();

					// Both points are in frame A, transform to world space
					return
					{
							.point_a = input.transform_a.transform(local_point_a),
							.point_b = input.transform_a.transform(local_point_b),
							.normal = Vec2::zero,
							.distance = 0,
					};
				};

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex.type == Simplex::Type::TRIANGLE)
				{
					return fallback_overlap_result();
				}

				// Ensure the search direction is numerically fit.
				if (d.dot(d) < std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon())
				{
					// This is unlikely but could lead to bad cycling.
					// The branch predictor seems to make this check have low cost.

					// The origin is probably contained by a line segment or triangle. 
					// Thus, the shapes are overlapped.

					// Must return overlap due to invalid normal.
					return fallback_overlap_result();
				}

				// Save the normal
				non_unit_normal = d;

				// Compute a tentative new simplex vertex using support points.
				// support = support(a, d) - support(b, -d)
				auto& vertex = simplex.vertices[static_cast<std::ptrdiff_t>(simplex.type)];
				vertex.index_a = input.proxy_a.find_support(d);
				vertex.w_a = input.proxy_a.points[vertex.index_a];
				vertex.index_b = local_proxy_b.find_support(-d);
				vertex.w_b = local_proxy_b.points[vertex.index_b];
				vertex.w = vertex.w_a - vertex.w_b;

				// Check for duplicate support points. 
				// This is the main termination criteria.
				const auto duplicate = std::ranges::contains(
					std::views::zip(save_a, save_b),
					true,
					[vertex](const auto& pair) noexcept -> bool
					{
						const auto [a, b] = pair;
						return a == vertex.index_a and b == vertex.index_b;
					}
				);

				// If we found a duplicate support point we must exit to avoid cycling.
				if (duplicate)
				{
					break;
				}

				// New vertex is valid and needed.
				simplex.type = static_cast<Simplex::Type>(static_cast<std::uint32_t>(simplex.type) + 1);
			}
		}

		// Prepare output
		auto normal = non_unit_normal.normalize();
		BPP_ASSERT(normal.normalized());
		normal = input.transform_a.rotation.rotate(normal);

		const auto [local_point_a, local_point_b] = simplex.compute_closest_points();

		Distance result
		{
				.point_a = input.transform_a.transform(local_point_a),
				.point_b = input.transform_a.transform(local_point_b),
				.normal = normal,
				.distance = local_point_a.distance(local_point_b),
		};

		// Cache the simplex
		inout_cache = simplex.cache();

		// Apply radii if requested
		if (input.use_radii)
		{
			const auto radius_a = input.proxy_a.radius;
			const auto radius_b = input.proxy_b.radius;
			result.distance = std::ranges::max(result.distance - radius_a - radius_b, 0.f);

			// Keep the closest points on perimeter even if overlapped, this way the points move smoothly.
			result.point_a = multiply_add(result.point_a, radius_a, normal);
			result.point_b = multiply_sub(result.point_b, radius_b, normal);
		}

		return result;
	}

	auto Distance::compute(const DistanceInput& input) noexcept -> Distance
	{
		auto cache = SimplexCache::zero;
		return compute(input, cache);
	}
}
