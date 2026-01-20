// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/shape_cast.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/utility/value_cast.hpp>

#include <box2dpp/shape/circle.hpp>
#include <box2dpp/shape/capsule.hpp>
#include <box2dpp/shape/polygon.hpp>
#include <box2dpp/shape/segment.hpp>

// ShapeCast::test(const ShapeCastPairInput& input)
#include <box2dpp/collision/simplex.hpp>
#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto ShapeProxy::create(const std::span<const Vec2> points, const float radius) noexcept -> ShapeProxy
	{
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = value_cast<std::uint32_t>(points.size()), .radius = radius};
		std::ranges::copy(points, result.points);

		return result;
	}

	auto ShapeProxy::create(const std::span<const Vec2> points, const float radius, const Transform& transform) noexcept -> ShapeProxy
	{
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = value_cast<std::uint32_t>(points.size()), .radius = radius};
		std::ranges::transform(
			points,
			result.points,
			[&transform](const Vec2& point) noexcept -> Vec2
			{
				return transform.transform(point);
			}
		);

		return result;
	}

	auto ShapeProxy::create(const std::span<const Vec2> points, const float radius, const Vec2& position, const Rotation& rotation) noexcept -> ShapeProxy
	{
		return create(points, radius, {.point = position, .rotation = rotation});
	}

	auto ShapeProxy::find_support(const Vec2& direction) const noexcept -> std::uint16_t
	{
		BPP_ASSERT(count != 0);

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

		return value_cast<std::uint16_t>(best_index);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Circle& circle) noexcept -> ShapeCast
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::create({&circle.center, 1}, circle.radius),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};
		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Capsule& capsule) noexcept -> ShapeCast
	{
		const Vec2 points[]{capsule.center1, capsule.center2};

		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::create({points, 2}, capsule.radius),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};
		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Polygon& polygon) noexcept -> ShapeCast
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::create({polygon.vertices, static_cast<std::size_t>(polygon.count)}, polygon.radius),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};
		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Segment& segment) noexcept -> ShapeCast
	{
		const Vec2 points[]{segment.point1, segment.point2};

		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::create({points, 2}, 0),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};
		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastPairInput& input) noexcept -> ShapeCast
	{
		// Compute tolerance
		const auto linear_slop = BPP_LINEAR_SLOP;
		const auto total_radius = input.proxy_a.radius + input.proxy_b.radius;
		const auto tolerance = linear_slop * .25f;

		auto target = std::ranges::max(linear_slop, total_radius - linear_slop);
		BPP_ASSERT(target > tolerance);

		// Prepare input for distance query
		SimplexCache cache{.count = 0, .index_a = {}, .index_b = {}};
		DistanceInput distance_input{.proxy_a = input.proxy_a, .proxy_b = input.proxy_b, .transform_a = input.transform_a, .transform_b = input.transform_b, .use_radii = false};

		float fraction = 0.f;

		const auto delta2 = input.translation_b;
		ShapeCast result{.normal = Vec2::zero, .point = Vec2::zero, .fraction = 0.f, .iterations = 0, .hit = false};

		for (std::uint32_t iteration = 0, max_iterations = 20; iteration < max_iterations; ++iteration)
		{
			result.iterations += 1;

			const auto output = DistanceOutput::compute(distance_input, cache);
			if (output.distance < target + tolerance)
			{
				if (iteration == 0)
				{
					if (input.can_encroach and output.distance > linear_slop * 2.f)
					{
						target = output.distance - linear_slop;
					}
					else
					{
						// Initial overlap
						result.hit = true;

						// Compute a common point
						const auto c1 = multiply_add(output.point_a, input.proxy_a.radius, output.normal);
						const auto c2 = multiply_add(output.point_b, -input.proxy_b.radius, output.normal);
						result.point = c1.lerp(c2, .5f);

						return result;
					}
				}
				else
				{
					// Regular hit
					BPP_ASSERT(output.distance > 0 and output.normal.normalized());

					result.normal = output.normal;
					result.point = multiply_add(output.point_a, input.proxy_a.radius, output.normal);
					result.fraction = fraction;
					result.hit = true;

					return result;
				}
			}

			BPP_ASSERT(output.distance > 0);
			BPP_ASSERT(output.normal.normalized());

			// Check if shapes are approaching each other
			const auto denominator = delta2.dot(output.normal);
			if (denominator >= 0)
			{
				// Miss
				return result;
			}

			// Advance sweep
			fraction += (target - output.distance) / denominator;
			if (fraction >= input.max_fraction)
			{
				// Miss
				return result;
			}

			distance_input.transform_b.point = multiply_add(input.transform_b.point, fraction, delta2);
		}

		// Failure!
		return result;
	}
}
