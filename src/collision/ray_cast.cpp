// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/ray_cast.hpp>

#include <algorithm>
#include <limits>
#include <ranges>

#include <box2dpp/shape/circle.hpp>
#include <box2dpp/shape/capsule.hpp>
#include <box2dpp/shape/polygon.hpp>
#include <box2dpp/shape/segment.hpp>

// RayCast::test(const RayCastInput& input, const Polygon& polygon) ==> ShapeCast::test(const ShapeCastPairInput& input)
#include <box2dpp/collision/shape_cast.hpp>

namespace box2dpp
{
	namespace
	{
		[[nodiscard]] auto make_initial_overlap(const RayCastInput& input) noexcept -> RayCastOutput
		{
			return {.normal = Vec2::zero, .point = input.origin, .fraction = 0.f};
		}
	}

	auto RayCastInput::valid() const noexcept -> bool
	{
		if (not origin.valid())
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

	auto RayCast::test(const RayCastInput& input, const Circle& circle) noexcept -> std::optional<RayCastOutput>
	{
		BPP_ASSERT(input.valid());
		BPP_ASSERT(circle.valid());

		// Shift ray so circle center is the origin
		const auto s = input.origin - circle.center;

		const auto r = circle.radius;
		const auto r2 = r * r;

		float length;
		const auto d = input.translation.normalize(length);

		// Zero length ray
		if (length < std::numeric_limits<float>::epsilon())
		{
			// Check if origin is inside circle
			if (s.length_squared() < r2)
			{
				return make_initial_overlap(input);
			}

			return std::nullopt;
		}

		// Find the closest point on infinite ray to circle center
		// Project vector s onto ray direction d
		const auto t = -s.dot(d);

		// Closest point on infinite ray to circle center: s + t * d
		const auto c = multiply_add(s, t, d);
		const auto c2 = c.dot(c);

		// If closest point is outside circle, no intersection
		if (c2 > r2)
		{
			return std::nullopt;
		}

		// Distance from the closest point to circle along ray direction
		const auto h = std::sqrt(r2 - c2);
		// First intersection point (entering the circle)
		const auto distance = t - h;
		// Convert to fraction of ray length
		const auto fraction = distance / length;

		// Check if intersection is within ray segment
		if (fraction < 0 or fraction > input.max_fraction)
		{
			// Check for initial overlap
			if (s.length_squared() < r2)
			{
				return make_initial_overlap(input);
			}

			return std::nullopt;
		}

		// hit point relative to center
		const auto hit_point = multiply_add(s, distance, d);
		const auto normal = hit_point.normalize();
		const auto point = multiply_add(circle.center, circle.radius, normal);

		return RayCastOutput{.normal = normal, .point = point, .fraction = fraction};
	}

	auto RayCast::test(const RayCastInput& input, const Capsule& capsule) noexcept -> std::optional<RayCastOutput>
	{
		BPP_ASSERT(input.valid());
		BPP_ASSERT(capsule.valid());

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

			// ray starts inside capsule
			return make_initial_overlap(input);
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
		if (std::abs(den) < std::numeric_limits<float>::epsilon())
		{
			// Ray is parallel to capsule and outside infinite length capsule
			return std::nullopt;
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
			return std::nullopt;
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
		const auto point = multiply_add(v1.lerp(v2, s1 / capsule_length), r, n);
		const auto fraction = s2 / ray_length;

		return RayCastOutput{.normal = n, .point = point, .fraction = fraction};
	}

	auto RayCast::test(const RayCastInput& input, const Polygon& polygon) noexcept -> std::optional<RayCastOutput>
	{
		BPP_ASSERT(input.valid());
		BPP_ASSERT(polygon.count >= 3);

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

				if (std::abs(denominator) < std::numeric_limits<float>::epsilon())
				{
					// Parallel and runs outside edge
					if (numerator < 0)
					{
						return std::nullopt;
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
					return std::nullopt;
				}
			}

			BPP_ASSERT(0 <= lower && lower <= input.max_fraction);

			if (index == std::numeric_limits<std::uint32_t>::max())
			{
				return make_initial_overlap(input);
			}

			const auto normal = polygon.normals[index];
			const auto point = multiply_add(input.origin, lower, d);
			const auto fraction = lower;

			return RayCastOutput{.normal = normal, .point = point, .fraction = fraction};
		}

		const ShapeCastPairInput pair_input
		{
				.proxy_a = ShapeProxy::from({polygon.vertices.data(), polygon.count}, polygon.radius),
				.proxy_b = ShapeProxy::from({&input.origin, 1}, 0),
				.transform_a = Transform::identity,
				.transform_b = Transform::identity,
				.translation_b = input.translation,
				.max_fraction = input.max_fraction,
				.can_encroach = false
		};

		if (const auto output = ShapeCast::test(pair_input);
			output.has_value())
		{
			const auto& o = *output;
			return RayCastOutput{.normal = o.normal, .point = o.point, .fraction = o.fraction};
		}

		return std::nullopt;
	}

	auto RayCast::test(const RayCastInput& input, const Segment& segment, const bool one_sided) noexcept -> std::optional<RayCastOutput>
	{
		BPP_ASSERT(input.valid());
		BPP_ASSERT(segment.valid());

		if (one_sided)
		{
			// Skip left-side collision
			if (const auto offset = (input.origin - segment.point1).cross(segment.point2 - segment.point1);
				offset < 0)
			{
				return std::nullopt;
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
		if (length < std::numeric_limits<float>::epsilon())
		{
			return std::nullopt;
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

		if (std::abs(denominator) < std::numeric_limits<float>::epsilon())
		{
			// parallel
			return std::nullopt;
		}

		const auto t = numerator / denominator;
		if (t < 0 or t > input.max_fraction)
		{
			// out of ray range
			return std::nullopt;
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
			return std::nullopt;
		}

		if (numerator > 0)
		{
			normal = -normal;
		}

		return RayCastOutput{.normal = normal, .point = p, .fraction = t};
	}
}
