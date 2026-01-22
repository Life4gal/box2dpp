// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/shape_cast.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/collision/simplex.hpp>
#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto ShapeCastInput::valid() const noexcept -> bool
	{
		if (not proxy.valid())
		{
			return false;
		}

		if (not translation.valid())
		{
			return false;
		}

		if (not box2dpp::valid(max_fraction) or max_fraction < 0 or max_fraction > 1)
		{
			return false;
		}

		return true;
	}

	auto ShapeCastPairInput::valid() const noexcept -> bool
	{
		if (not proxy_a.valid() or not proxy_b.valid())
		{
			return false;
		}

		if (not transform_a.valid() or not transform_b.valid())
		{
			return false;
		}

		if (not translation_b.valid())
		{
			return false;
		}

		if (not box2dpp::valid(max_fraction) or max_fraction < 0 or max_fraction > 1)
		{
			return false;
		}

		return true;
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Circle& circle) noexcept -> std::optional<ShapeCastOutput>
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::from(circle),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};

		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Capsule& capsule) noexcept -> std::optional<ShapeCastOutput>
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::from(capsule),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};

		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Polygon& polygon) noexcept -> std::optional<ShapeCastOutput>
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::from(polygon),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};

		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastInput& input, const Segment& segment) noexcept -> std::optional<ShapeCastOutput>
	{
		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::from(segment),
				.proxy_b = input.proxy,
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = input.can_encroach
		};

		return test(pair_input);
	}

	auto ShapeCast::test(const ShapeCastPairInput& input) noexcept -> std::optional<ShapeCastOutput>
	{
		BPP_ASSERT(input.valid());

		if (input.translation_b.length_squared() < std::numeric_limits<float>::epsilon())
		{
			// Zero translation, perform static overlap test
			const DistanceInput distance_input
			{
					.proxy_a = input.proxy_a,
					.proxy_b = input.proxy_b,
					.transform_a = input.transform_a,
					.transform_b = input.transform_b,
					.use_radii = true
			};
			if (const auto distance = Distance::compute(distance_input);
				distance.distance <= 0.0f)
			{
				return ShapeCastOutput{.normal = Vec2::zero, .point = (distance.point_a + distance.point_b) * 0.5f, .fraction = 0.0f};
			}
			return std::nullopt;
		}

		// Compute tolerance
		const auto linear_slop = BPP_LINEAR_SLOP;
		const auto total_radius = input.proxy_a.radius + input.proxy_b.radius;
		const auto tolerance = linear_slop * .25f;

		const auto delta = input.translation_b;

		// Initial target separation (accounts for shape radii)
		auto target = std::ranges::max(linear_slop, total_radius - linear_slop);
		BPP_ASSERT(target > tolerance);

		// Prepare input for distance query
		DistanceInput distance_input
		{
				.proxy_a = input.proxy_a,
				.proxy_b = input.proxy_b,
				.transform_a = input.transform_a,
				.transform_b = input.transform_b,
				.use_radii = false
		};
		auto cache = SimplexCache::zero;
		float fraction = 0.f;

		for (std::uint32_t iteration = 0; iteration < BPP_COLLISION_DISTANCE_MAX_ITERATIONS; ++iteration)
		{
			// Compute current distance between shapes
			const auto distance = Distance::compute(distance_input, cache);

			// Check if we're close enough to the target separation
			if (distance.distance < target + tolerance)
			{
				if (iteration == 0)
				{
					// First iteration - handle initial proximity

					if (input.can_encroach and distance.distance > linear_slop * 2.f)
					{
						// Allow shapes to get slightly closer
						target = distance.distance - linear_slop;
					}
					else
					{
						// Initial overlap or very close proximity
						// Compute a representative contact point

						const auto point_a = multiply_add(distance.point_a, input.proxy_a.radius, distance.normal);
						const auto point_b = multiply_add(distance.point_b, -input.proxy_b.radius, distance.normal);
						const auto contact_point = (point_a + point_b) * .5f;

						return ShapeCastOutput{.normal = Vec2::zero, .point = contact_point, .fraction = 0};
					}
				}
				else
				{
					// Regular hit during sweep
					BPP_ASSERT(distance.distance > 0 and distance.normal.normalized());

					// Compute contact point on surface of shape A
					const auto contact_point = multiply_add(distance.point_a, input.proxy_a.radius, distance.normal);

					return ShapeCastOutput{.normal = distance.normal, .point = contact_point, .fraction = fraction};
				}
			}

			BPP_ASSERT(distance.distance > 0);
			BPP_ASSERT(distance.normal.normalized());

			// Check if shapes are moving apart
			const float approach_speed = delta.dot(distance.normal);
			if (approach_speed >= 0)
			{
				// Shapes are moving apart or parallel, no collision
				return std::nullopt;
			}

			// Compute required movement to reach target separation
			const float required_movement = (target - distance.distance) / approach_speed;
			BPP_ASSERT(required_movement > 0);

			// Advance the sweep
			fraction += required_movement;
			if (fraction >= input.max_fraction)
			{
				// Would collide beyond max_fraction
				return std::nullopt;
			}

			// Update transform for next iteration
			distance_input.transform_b.point = multiply_add(input.transform_b.point, fraction, delta);
		}

		// Algorithm failed to converge or max iterations exceeded
		return std::nullopt;
	}
}
