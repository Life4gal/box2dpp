// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/cast.hpp>

#include <algorithm>
#include <limits>
#include <ranges>

#include <box2dpp/collision/circle.hpp>
#include <box2dpp/collision/capsule.hpp>
#include <box2dpp/collision/polygon.hpp>
#include <box2dpp/collision/segment.hpp>

#include <box2dpp/collision/simplex.hpp>
#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto RayCastInput::valid() const noexcept -> bool
	{
		return
				origin.valid() and
				translation.valid() and
				box2dpp::valid(max_fraction) and max_fraction >= 0.f and max_fraction < BPP_HUGE;
	}

	auto ShapeProxy::create(const std::span<const Vec2> points, const float radius) noexcept -> ShapeProxy
	{
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = static_cast<std::uint32_t>(points.size()), .radius = radius};
		std::ranges::copy(points, result.points);

		return result;
	}

	auto ShapeProxy::create(const std::span<const Vec2> points, const float radius, const Transform& transform) noexcept -> ShapeProxy
	{
		BPP_ASSERT(points.size() <= BPP_MAX_POLYGON_VERTICES);

		ShapeProxy result{.points = {}, .count = static_cast<std::uint32_t>(points.size()), .radius = radius};
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

		BPP_ASSERT(std::cmp_less(best_index, std::numeric_limits<std::uint16_t>::max()));
		return static_cast<std::uint16_t>(best_index);
	}

	namespace
	{
		[[nodiscard]] constexpr auto cast_missed() noexcept -> CastOutput
		{
			return {.normal = Vec2::zero, .point = Vec2::zero, .fraction = 0.f, .iterations = 0, .hit = false};
		}
	}

	auto CastOutput::test(const RayCastInput& input, const Circle& circle) noexcept -> CastOutput
	{
		BPP_ASSERT(input.valid());

		// Shift ray so circle center is the origin
		const auto s = input.origin - circle.center;

		const auto r = circle.radius;
		const auto r2 = r * r;

		float length;
		const auto d = input.translation.normalize(length);
		if (length == 0) // NOLINT(clang-diagnostic-float-equal)
		{
			// zero length ray
			if (s.length_squared() < r2)
			{
				// initial overlap
				return {.normal = Vec2::zero, .point = input.origin, .fraction = 0.f, .iterations = 0, .hit = true};
			}

			return cast_missed();
		}

		// Find the closest point on ray to origin

		// solve: dot(s + t * d, d) = 0
		const auto t = -s.dot(d);

		// c is the closest point on the line to the origin
		const auto c = multiply_add(s, t, d);
		const auto c2 = c.dot(c);

		if (c2 > r2)
		{
			// closest point is outside the circle
			return cast_missed();
		}

		// Pythagoras
		const auto h = std::sqrt(r2 - c2);
		const auto fraction = t - h;

		if (fraction < 0 or input.max_fraction * length < fraction)
		{
			// intersection is point outside the range of the ray segment

			if (s.length_squared() < r2)
			{
				// initial overlap
				return {.normal = Vec2::zero, .point = input.origin, .fraction = 0.f, .iterations = 0, .hit = true};
			}

			return cast_missed();
		}

		// hit point relative to center
		const auto hit_point = multiply_add(s, fraction, d);
		const auto normal = hit_point.normalize();

		return {.normal = normal, .point = multiply_add(circle.center, circle.radius, normal), .fraction = fraction / length, .iterations = 0, .hit = true};
	}

	auto CastOutput::test(const RayCastInput& input, const Capsule& capsule) noexcept -> CastOutput
	{
		BPP_ASSERT(input.valid());

		const auto v1 = capsule.center1;
		const auto v2 = capsule.center2;
		const auto e = v2 - v1;

		float capsule_length;
		const auto a = e.normalize(capsule_length);
		if (capsule_length < std::numeric_limits<float>::epsilon())
		{
			// Capsule is really a circle
			const Circle circle{.center = v1, .radius = capsule.radius};
			return test(input, circle);
		}

		const auto p1 = input.origin;
		const auto d = input.translation;

		const auto r = capsule.radius;
		const auto r2 = r * r;

		// Ray from capsule start to ray start
		const auto q = p1 - v1;
		const auto qa = q.dot(a);

		// Vector to ray start that is perpendicular to capsule axis
		const auto qp = multiply_add(q, -qa, a);

		// Does the ray start within the infinite length capsule?
		if (qp.dot(qp) < r2)
		{
			if (qa < 0)
			{
				// start point behind capsule segment
				const Circle circle{.center = v1, .radius = capsule.radius};
				return test(input, circle);
			}

			if (qa > capsule_length)
			{
				// start point ahead of capsule segment
				const Circle circle{.center = v2, .radius = capsule.radius};
				return test(input, circle);
			}

			// ray starts inside capsule -> no hit
			return {.normal = Vec2::zero, .point = input.origin, .fraction = 0, .iterations = 0, .hit = true};
		}

		// Perpendicular to capsule axis, pointing right
		Vec2 n{.x = a.y, .y = -a.x};

		float ray_length;
		const auto u = d.normalize(ray_length);

		// Intersect ray with infinite length capsule
		// v1 + radius * n + s1 * a = p1 + s2 * u
		// v1 - radius * n + s1 * a = p1 + s2 * u

		// s1 * a - s2 * u = b
		// b = q - radius * ap
		// or
		// b = q + radius * ap

		// Cramer's rule [a -u]
		const auto den = a.cross(-u);
		if (-std::numeric_limits<float>::epsilon() < den and den < std::numeric_limits<float>::epsilon())
		{
			// Ray is parallel to capsule and outside infinite length capsule
			return cast_missed();
		}

		const auto b1 = multiply_sub(q, r, n);
		const auto b2 = multiply_add(q, r, n);

		const auto inv_den = 1.f / den;

		float s2;
		Vec2 b;
		{
			// Cramer's rule [a b1]
			const auto s21 = a.cross(b1) * inv_den;

			// Cramer's rule [a b2]
			const auto s22 = a.cross(b2) * inv_den;

			if (s21 < s22)
			{
				s2 = s21;
				b = b1;
			}
			else
			{
				s2 = s22;
				b = b2;

				n = -n;
			}
		}

		if (s2 < 0 or s2 > input.max_fraction * ray_length)
		{
			return cast_missed();
		}

		// Cramer's rule [b -u]
		const auto s1 = b.cross(-u) * inv_den;
		if (s1 < 0)
		{
			// ray passes behind capsule segment
			const Circle circle{.center = v1, .radius = capsule.radius};
			return test(input, circle);
		}
		if (s1 > capsule_length)
		{
			// ray passes ahead of capsule segment
			const Circle circle{.center = v2, .radius = capsule.radius};
			return test(input, circle);
		}

		// ray hits capsule side
		return {.normal = n, .point = v1.lerp(v2, s1 / capsule_length) + r * n, .fraction = s2 / ray_length, .iterations = 0, .hit = true};
	}

	auto CastOutput::test(const RayCastInput& input, const Polygon& polygon) noexcept -> CastOutput
	{
		BPP_ASSERT(input.valid());

		if (polygon.radius == 0) // NOLINT(clang-diagnostic-float-equal)
		{
			// Shift all math to first vertex since the polygon may be far from the origin.
			const auto base = polygon.vertices[0];

			const auto p1 = input.origin - base;
			const auto d = input.translation;

			float lower = 0;
			float upper = input.max_fraction;

			auto index = std::numeric_limits<std::uint32_t>::max();
			for (std::uint32_t edge_index = 0; edge_index < polygon.count; ++edge_index)
			{
				const auto& edge_vertex = polygon.vertices[edge_index];
				const auto& edge_normal = polygon.normals[edge_index];

				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0
				const auto vertex = edge_vertex - base;
				const auto numerator = edge_normal.dot(vertex - p1);
				const auto denominator = edge_normal.dot(d);

				if (denominator == 0) // NOLINT(clang-diagnostic-float-equal)
				{
					// Parallel and runs outside edge
					if (numerator < 0)
					{
						return cast_missed();
					}
				}
				else
				{
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower > numerator.
					if (denominator < 0 and numerator < lower * denominator)
					{
						// Increase lower.
						// The segment enters this half-space.
						lower = numerator / denominator;
						index = edge_index;
					}
					else if (denominator > 0 and numerator < upper * denominator)
					{
						// Decrease upper.
						// The segment exits this half-space.
						upper = numerator / denominator;
					}
				}

				if (upper < lower)
				{
					// Ray misses
					return cast_missed();
				}
			}

			BPP_ASSERT(0 <= lower && lower <= input.max_fraction);

			if (index == std::numeric_limits<std::uint32_t>::max())
			{
				// initial overlap
				return {.normal = Vec2::zero, .point = input.origin, .fraction = 0, .iterations = 0, .hit = true};
			}

			return {.normal = polygon.normals[index], .point = multiply_add(input.origin, lower, d), .fraction = lower, .iterations = 0, .hit = true};
		}

		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::create({polygon.vertices, polygon.count}, polygon.radius),
				.proxy_b = ShapeProxy::create({&input.origin, 1}, 0),
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = false
		};
		return test(pair_input);
	}

	auto CastOutput::test(const RayCastInput& input, const Segment& segment, const bool one_sided) noexcept -> CastOutput
	{
		BPP_ASSERT(input.valid());

		if (one_sided)
		{
			// Skip left-side collision
			if (const auto offset = (input.origin - segment.point1).cross(segment.point2 - segment.point1);
				offset < 0)
			{
				return cast_missed();
			}
		}

		// Put the ray into the edge's frame of reference.
		const auto p1 = input.origin;
		const auto d = input.translation;

		const auto v1 = segment.point1;
		const auto v2 = segment.point2;
		const auto e = v2 - v1;

		float length;
		const auto e_normalized = e.normalize(length);
		if (length == 0) // NOLINT(clang-diagnostic-float-equal)
		{
			return cast_missed();
		}

		// Normal points to the right, looking from v1 towards v2
		auto normal = e_normalized.right_perpendicular();

		// Intersect ray with infinite segment using normal
		// Similar to intersecting a ray with an infinite plane
		// p = p1 + t * d
		// dot(normal, p - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		const auto numerator = normal.dot(v1 - p1);
		const auto denominator = normal.dot(d);

		if (denominator == 0) // NOLINT(clang-diagnostic-float-equal)
		{
			// parallel
			return cast_missed();
		}

		const auto t = numerator / denominator;
		if (t < 0 or t > input.max_fraction)
		{
			// out of ray range
			return cast_missed();
		}

		// Intersection point on infinite segment
		const auto p = multiply_add(p1, t, d);

		// Compute position of p along segment
		// p = v1 + s * e
		// s = dot(p - v1, e) / dot(e, e)
		const auto s = (p - v1).dot(e_normalized);
		if (s < 0 or s > length)
		{
			// out of segment range
			return cast_missed();
		}

		if (numerator > 0)
		{
			normal = -normal;
		}

		return {.normal = normal, .point = p, .fraction = t, .iterations = 0, .hit = true};
	}

	auto CastOutput::test(const ShapeCastInput& input, const Circle& circle) noexcept -> CastOutput
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

	auto CastOutput::test(const ShapeCastInput& input, const Capsule& capsule) noexcept -> CastOutput
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

	auto CastOutput::test(const ShapeCastInput& input, const Polygon& polygon) noexcept -> CastOutput
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

	auto CastOutput::test(const ShapeCastInput& input, const Segment& segment) noexcept -> CastOutput
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

	auto CastOutput::test(const ShapeCastPairInput& input) noexcept -> CastOutput
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
		CastOutput result{.normal = Vec2::zero, .point = Vec2::zero, .fraction = 0.f, .iterations = 0, .hit = false};

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
