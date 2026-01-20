// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/math/transform.hpp>

namespace box2dpp
{
	auto Transform::valid() const noexcept -> bool
	{
		return point.valid() and rotation.valid();
	}

	auto Transform::transform(const Vec2& p) const noexcept -> Vec2
	{
		// Apply rotation then translation: p' = R * p + t
		return rotation.rotate(p) + point;
	}

	auto Transform::inv_transform(const Vec2& p) const noexcept -> Vec2
	{
		// Inverse transform: p' = R⁻¹ * (p - t)
		return rotation.inv_rotate(p - point);
	}

	auto Transform::transform_vector(const Vec2& v) const noexcept -> Vec2
	{
		// Transform direction vector (ignores translation)
		return rotation.rotate(v);
	}

	auto Transform::inv_transform_vector(const Vec2& v) const noexcept -> Vec2
	{
		// Inverse transform direction vector
		return rotation.inv_rotate(v);
	}

	auto Transform::multiply(const Transform& other) const noexcept -> Transform
	{
		// Compose transforms: (R_a, t_a) × (R_b, t_b) = (R_a × R_b, R_a × t_b + t_a)
		// Apply other first, then this

		// R_a × t_b + t_a
		const auto p = transform(other.point);
		// R_a × R_b
		const auto r = rotation.multiply(other.rotation);

		return {.point = p, .rotation = r};
	}

	auto Transform::multiply_by_inv(const Transform& other) const noexcept -> Transform
	{
		// this⁻¹ × other = transform from other's frame to this frame
		// (R_a⁻¹ × R_b, R_a⁻¹ × (t_b - t_a))

		// R_a⁻¹ × (t_b - t_a)
		const auto p = inv_transform(other.point);
		// R_a⁻¹ × R_b
		const auto r = rotation.multiply_by_inv(other.rotation);

		return {.point = p, .rotation = r};
	}

	auto Transform::inv_multiply(const Transform& other) const noexcept -> Transform
	{
		// other × this⁻¹ = transform from this frame to other frame
		//  (R_b × R_a⁻¹, t_b - (R_b × R_a⁻¹) × t_a)

		// R_b × R_a⁻¹
		const auto r = rotation.inv_multiply(other.rotation);
		// t_b - (R_b × R_a⁻¹) × t_a
		const auto p = other.point - r.rotate(point);

		return {.point = p, .rotation = r};
	}
}
