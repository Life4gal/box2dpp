// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/mass_data.hpp>

#include <algorithm>
#include <ranges>
#include <numbers>

#include <box2dpp/shape/circle.hpp>
#include <box2dpp/shape/capsule.hpp>
#include <box2dpp/shape/polygon.hpp>

namespace box2dpp
{
	auto MassData::compute(const Circle& circle, const float density) noexcept -> MassData
	{
		const auto r2 = circle.radius * circle.radius;
		const auto mass = std::numbers::pi_v<float> * r2 * density;
		// inertia about the center of mass
		const auto inertia = mass * .5f * r2;

		return {.mass = mass, .center = circle.center, .rotational_inertia = inertia};
	}

	auto MassData::compute(const Capsule& capsule, const float density) noexcept -> MassData
	{
		const auto r = capsule.radius;
		const auto r2 = capsule.radius * capsule.radius;
		const auto length = capsule.center2.distance(capsule.center1);
		const auto length_2 = capsule.center2.distance_squared(capsule.center1);

		const auto circle_mass = (std::numbers::pi_v<float> * r2) * density;
		const auto box_mass = (2 * r * length) * density;

		const auto mass = circle_mass + box_mass;
		const auto center = (capsule.center1 + capsule.center2) / 2;

		// two offset half circles, both halves add up to full circle and each half is offset by half-length
		// semicircle centroid = 4 r / 3 pi
		// Need to apply parallel-axis theorem twice:
		// 1. shift semicircle centroid to origin
		// 2. shift semicircle to box end
		// m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		// I verified this formula by computing the convex hull of a 128 vertex capsule

		// half circle centroid
		const auto lc = 4.f * r / (3.f * std::numbers::pi_v<float>);
		// half-length of rectangular portion of capsule
		const auto h = .5f * length;
		const auto h2 = h * h;

		const auto circle_inertia = circle_mass * (.5f * r2 + h2 + 2.f * h * lc);
		const auto box_inertia = box_mass * (4.f * r2 + length_2) / 12.f;
		const auto inertia = circle_inertia + box_inertia;

		return {.mass = mass, .center = center, .rotational_inertia = inertia};
	}

	auto MassData::compute(const Polygon& polygon, const float density) noexcept -> MassData
	{
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals for each triangle of the polygon. 
		// To evaluate the integral for a single triangle, 
		// we make a change of variables to the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		BPP_ASSERT(polygon.count > 0);

		if (polygon.count == 1)
		{
			const Circle circle{.center = polygon.vertices[0], .radius = polygon.radius};

			return compute(circle, density);
		}

		if (polygon.count == 2)
		{
			const Capsule capsule{.center1 = polygon.vertices[0], .center2 = polygon.vertices[1], .radius = polygon.radius};

			return compute(capsule, density);
		}

		Vec2 vertices[BPP_MAX_POLYGON_VERTICES];

		if (polygon.radius > 0)
		{
			// Approximate mass of rounded polygons by pushing out the vertices.
			for (std::uint32_t index = 0; index < polygon.count; ++index)
			{
				const auto index_prev = index == 0 ? polygon.count - 1 : index - 1;

				const auto& normal = polygon.normals[index];
				const auto& normal_prev = polygon.normals[index_prev];

				const auto mid = (normal_prev + normal).normalize();
				vertices[index] = polygon.vertices[index] + std::numbers::sqrt2_v<float> * polygon.radius * mid;
			}
		}
		else
		{
			std::ranges::copy(std::views::counted(polygon.vertices, polygon.count), vertices);
		}

		Vec2 center{.x = 0, .y = 0};
		float area = 0;
		float inertia = 0;

		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		const auto r = vertices[0];

		for (std::uint32_t index_v1 = 1; index_v1 < polygon.count - 1; ++index_v1)
		{
			const auto index_v2 = index_v1 + 1;

			const auto& vertex_1 = vertices[index_v1];
			const auto& vertex_2 = vertices[index_v2];

			// Triangle edges
			const auto e1 = vertex_1 - r;
			const auto e2 = vertex_2 - r;

			const auto d = e1.cross(e2);
			const auto triangle_area = d * .5f;

			// Area weighted centroid, r at origin
			constexpr auto inv_3 = 1.f / 3.f;
			center += triangle_area * inv_3 * (e1 + e2);

			area += triangle_area;

			const auto x = e1.x * e1.x + e1.x * e2.x + e2.x * e2.x;
			const auto y = e1.y * e1.y + e1.y * e2.y + e2.y * e2.y;
			inertia += (.25f * inv_3 * d) * (x + y);
		}

		// Total mass
		const auto mass = area * density;

		// Center of mass, shift back from origin at r
		BPP_ASSERT(area > std::numeric_limits<float>::epsilon());
		const auto inv_area = 1.f / area;
		center *= inv_area;

		// Inertia tensor relative to the local origin (point s).
		inertia *= density;
		// Shift inertia to center of mass
		inertia -= mass * center.dot(center);

		// If this goes negative we are hosed
		BPP_ASSERT(inertia >= 0);

		return {.mass = mass, .center = center + r, .rotational_inertia = inertia};
	}
}
