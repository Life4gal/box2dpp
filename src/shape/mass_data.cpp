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

		// area = πr², mass = density * area
		const auto mass = std::numbers::pi_v<float> * r2 * density;

		// Moment of inertia about center for disk: (1/2)mr²
		const auto inertia = mass * .5f * r2;

		return {.mass = mass, .center = circle.center, .rotational_inertia = inertia};
	}

	auto MassData::compute(const Capsule& capsule, const float density) noexcept -> MassData
	{
		const auto r = capsule.radius;
		const auto r2 = capsule.radius * capsule.radius;

		// Distance between capsule centers
		const auto length = capsule.center2.distance(capsule.center1);
		const auto length_2 = capsule.center2.distance_squared(capsule.center1);

		// Mass of circular ends (full circle)
		const auto circle_mass = (std::numbers::pi_v<float> * r2) * density;
		// Mass of rectangular middle section
		const auto box_mass = (2 * r * length) * density;
		// Total mass
		const auto mass = circle_mass + box_mass;

		// Centroid is midpoint between centers
		const auto center = (capsule.center1 + capsule.center2) / 2;

		// Complex inertia calculation using parallel axis theorem
		// Circle inertia shifted from its centroid to capsule end, then to center
		// Semicircle centroid offset
		const auto lc = 4.f * r / (3.f * std::numbers::pi_v<float>);

		// Half-length of rectangle
		const auto h = .5f * length;
		const auto h2 = h * h;

		// Circle inertia about capsule center
		const auto circle_inertia = circle_mass * (.5f * r2 + h2 + 2.f * h * lc);
		// Rectangle inertia about its center (thin rod formula modified for rectangle)
		const auto box_inertia = box_mass * (4.f * r2 + length_2) / 12.f;
		// Total inertia
		const auto inertia = circle_inertia + box_inertia;

		return {.mass = mass, .center = center, .rotational_inertia = inertia};
	}

	auto MassData::compute(const Polygon& polygon, const float density) noexcept -> MassData
	{
		BPP_ASSERT(polygon.count > 0);

		// Single vertex -> treat as circle with given radius
		if (polygon.count == 1)
		{
			const Circle circle{.center = polygon.vertices[0], .radius = polygon.radius};

			return compute(circle, density);
		}

		// Two vertices -> treat as capsule
		if (polygon.count == 2)
		{
			const Capsule capsule{.center1 = polygon.vertices[0], .center2 = polygon.vertices[1], .radius = polygon.radius};

			return compute(capsule, density);
		}

		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> vertices;

		// For rounded polygons, expand vertices outward by radius
		if (polygon.radius > 0)
		{
			// Expand polygon for rounded corners by moving vertices along angle bisectors
			for (std::uint32_t index = 0; index < polygon.count; ++index)
			{
				const auto index_prev = index == 0 ? polygon.count - 1 : index - 1;

				const auto& normal = polygon.normals[index];
				const auto& normal_prev = polygon.normals[index_prev];

				// Bisector of adjacent edge normals
				const auto mid = (normal_prev + normal).normalize();

				// Expand vertex by √2 * radius along bisector (for 45° rounded corners)
				vertices[index] = polygon.vertices[index] + std::numbers::sqrt2_v<float> * polygon.radius * mid;
			}
		}
		else
		{
			// Copy vertices for non-rounded polygon
			std::ranges::copy(std::views::counted(polygon.vertices.data(), polygon.count), vertices.data());
		}

		// Compute mass properties by triangulating polygon
		// Using reference point method for numerical stability
		Vec2 center{.x = 0, .y = 0};
		float area = 0;
		float inertia = 0;

		// Use first vertex as reference point
		const auto r = vertices[0];

		// Triangulate polygon from reference point
		for (std::uint32_t index_v1 = 1; index_v1 < polygon.count - 1; ++index_v1)
		{
			const auto index_v2 = index_v1 + 1;

			const auto& vertex_1 = vertices[index_v1];
			const auto& vertex_2 = vertices[index_v2];

			// Triangle edges from reference point
			const auto e1 = vertex_1 - r;
			const auto e2 = vertex_2 - r;

			// Signed area of triangle (2D cross product)
			const auto d = e1.cross(e2);
			const auto triangle_area = d * .5f;

			// Area-weighted centroid contribution
			constexpr auto inv_3 = 1.f / 3.f;
			center += triangle_area * inv_3 * (e1 + e2);

			area += triangle_area;

			// Inertia contribution for triangle about reference point
			const auto x = e1.x * e1.x + e1.x * e2.x + e2.x * e2.x;
			const auto y = e1.y * e1.y + e1.y * e2.y + e2.y * e2.y;
			inertia += (.25f * inv_3 * d) * (x + y);
		}

		// Compute total mass
		const auto mass = area * density;

		// Compute centroid (shift from reference point)
		BPP_ASSERT(area > std::numeric_limits<float>::epsilon());
		const auto inv_area = 1.f / area;
		center *= inv_area;

		// Adjust inertia from reference point to centroid
		// Convert to mass-weighted
		inertia *= density;
		// Parallel axis theorem
		inertia -= mass * center.dot(center);

		// Inertia should be non-negative
		BPP_ASSERT(inertia >= 0);

		// Shift centroid back from reference point
		return {.mass = mass, .center = center + r, .rotational_inertia = inertia};
	}

	auto MassData::valid() const noexcept -> bool
	{
		// Check for positive mass and inertia, finite values
		return
				mass > 0.0f and
				// Can be zero for point mass
				rotational_inertia >= 0.0f and
				box2dpp::valid(mass) and
				box2dpp::valid(rotational_inertia) and
				center.valid();
	}
}
