// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/distance.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/collision/segment.hpp>

namespace box2dpp
{
	auto DistanceOutput::compute(const DistanceInput& input, SimplexCache& inout_cache, const std::span<Simplex> debug_simplexes) noexcept -> DistanceOutput
	{
		BPP_ASSERT(input.proxy_a.count > 0 and input.proxy_b.count > 0);
		BPP_ASSERT(input.proxy_a.radius >= 0 and input.proxy_b.radius >= 0);

		// Get proxy B in frame A to avoid further transforms in the main loop.
		// This is still a performance gain at 8 points.
		ShapeProxy local_proxy_b{.points = {}, .count = input.proxy_b.count, .radius = input.proxy_b.radius};
		{
			const auto transform = input.transform_a.inv_multiply(input.transform_b);

			std::ranges::transform(
				std::views::counted(input.proxy_b.points, input.proxy_b.count),
				local_proxy_b.points,
				[&transform](const Vec2& point) noexcept -> Vec2
				{
					return transform.transform(point);
				}
			);
		}

		// Initialize the simplex
		auto simplex = Simplex::create(inout_cache, input.proxy_a, local_proxy_b);
		BPP_ASSERT(simplex.count <= 3);

		std::uint32_t simplex_index = 0;
		if (not debug_simplexes.empty())
		{
			debug_simplexes[0] = simplex;
			simplex_index += 1;
		}

		auto non_unit_normal = Vec2::zero;

		// Main iteration loop. 
		// All computations are done in frame A.
		std::uint32_t iteration = 0;
		{
			constexpr std::uint32_t max_iterations = 20;
			// These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
			std::uint16_t save_a[3];
			std::uint16_t save_b[3];

			while (iteration < max_iterations)
			{
				// Copy simplex so we can identify duplicates.
				const auto save_count = simplex.count;
				for (std::uint32_t i = 0; i < save_count; ++i)
				{
					save_a[i] = simplex.vertices[i].index_a;
					save_b[i] = simplex.vertices[i].index_b;
				}

				const auto d = [&]() noexcept -> Vec2
				{
					switch (simplex.count)
					{
						case 1:
						{
							return -simplex.vertices[0].w;
						}
						case 2:
						{
							return simplex.solve2();
						}
						case 3:
						{
							return simplex.solve3();
						}
						default:
						{
							BPP_COMPILER_UNREACHABLE();
						}
					}
				}();

				const auto fallback_overlap_result = [&] noexcept -> DistanceOutput
				{
					// Overlap
					Vec2 local_point_a;
					Vec2 local_point_b;
					simplex.compute_witness_points(local_point_a, local_point_b);

					return
					{
							.point_a = input.transform_a.transform(local_point_a),
							.point_b = input.transform_b.transform(local_point_b),
							.normal = Vec2::zero,
							.distance = 0,
							.iterations = 0,
							.simplex_count = 0
					};
				};

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex.count == 3)
				{
					return fallback_overlap_result();
				}

#if BPP_COMPILER_DEBUG
				if (not debug_simplexes.empty())
				{
					debug_simplexes[simplex_index] = simplex;
					simplex_index += 1;
				}
#endif

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
				auto& vertex = simplex.vertices[simplex.count];
				vertex.index_a = input.proxy_a.find_support(d);
				vertex.w_a = input.proxy_a.points[vertex.index_a];
				vertex.index_b = local_proxy_b.find_support(-d);
				vertex.w_b = local_proxy_b.points[vertex.index_b];
				vertex.w = vertex.w_a - vertex.w_b;

				// Iteration count is equated to the number of support point calls.
				iteration += 1;

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
				simplex.count += 1;
			}
		}

#if BPP_COMPILER_DEBUG
		if (not debug_simplexes.empty())
		{
			debug_simplexes[simplex_index] = simplex;
			simplex_index += 1;
		}
#endif

		// Prepare output
		auto normal = non_unit_normal.normalize();
		BPP_ASSERT(normal.normalized());
		normal = input.transform_a.rotation.rotate(normal);

		Vec2 local_point_a;
		Vec2 local_point_b;
		simplex.compute_witness_points(local_point_a, local_point_b);

		DistanceOutput result
		{
				.point_a = input.transform_a.transform(local_point_a),
				.point_b = input.transform_a.transform(local_point_b),
				.normal = normal,
				.distance = local_point_a.distance(local_point_b),
				.iterations = iteration,
				.simplex_count = simplex_index
		};

		// Cache the simplex
		inout_cache = SimplexCache::create(simplex);

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

	auto DistanceOutput::compute(const DistanceInput& input, const std::span<Simplex> debug_simplexes) noexcept -> DistanceOutput
	{
		SimplexCache cache{.count = 0, .index_a = {}, .index_b = {}};
		return compute(input, cache, debug_simplexes);
	}

	auto SegmentDistanceOutput::compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistanceOutput
	{
		return compute(segment1.point1, segment1.point2, segment2.point1, segment2.point2);
	}

	auto SegmentDistanceOutput::compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistanceOutput
	{
		const auto d1 = q1 - p1;
		const auto d2 = q2 - p2;
		const auto r = p1 - p2;
		const auto dd1 = d1.dot(d1);
		const auto dd2 = d2.dot(d2);
		const auto rd1 = r.dot(d1);
		const auto rd2 = r.dot(d2);

		constexpr auto epsilon2 = std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon();

		float fraction1;
		float fraction2;

		if (dd1 < epsilon2 or dd2 < epsilon2)
		{
			// Handle all degeneracies
			if (dd1 >= epsilon2)
			{
				// Segment 2 is degenerate
				fraction1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				fraction2 = 0;
			}
			else if (dd2 >= epsilon2)
			{
				fraction1 = 0;
				fraction2 = std::ranges::clamp(rd2 / dd2, 0.f, 1.f);
			}
			else
			{
				fraction1 = 0;
				fraction2 = 0;
			}
		}
		else
		{
			// Non-degenerate segments
			const auto d12 = d1.dot(d2);
			const auto denominator = dd1 * dd2 - d12 * d12;

			// Fraction on segment 1
			float f1 = 0;
			if (denominator != 0) // NOLINT(clang-diagnostic-float-equal)
			{
				// not parallel
				f1 = std::ranges::clamp((d12 * rd2 - dd2 * rd1) / denominator, 0.f, 1.f);
			}

			// Compute point on segment 2 closest to p1 + f1 * d1
			auto f2 = (d12 * f1 + rd2) / dd2;

			// Clamping of segment 2 requires a do-over on segment 1
			if (f2 < 0)
			{
				f1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				f2 = 0;
			}
			else if (f2 > 1)
			{
				f1 = std::ranges::clamp((d12 - rd1) / dd1, 0.f, 1.f);
				f2 = 1;
			}

			fraction1 = f1;
			fraction2 = f2;
		}

		const auto closest1 = multiply_add(p1, fraction1, d1);
		const auto closest2 = multiply_add(p2, fraction2, d2);
		const auto distance_squared = closest1.distance_squared(closest2);

		return {.closest1 = closest1, .closest2 = closest2, .fraction1 = fraction1, .fraction2 = fraction2, .distance_squared = distance_squared};
	}
}
