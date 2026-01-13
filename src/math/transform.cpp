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
		return rotation.rotate(p) + point;
	}

	auto Transform::inv_transform(const Vec2& p) const noexcept -> Vec2
	{
		return rotation.inv_rotate(p - point);
	}

	auto Transform::multiply(const Transform& other) const noexcept -> Transform
	{
		const auto p = transform(other.point);
		const auto r = rotation.multiply(other.rotation);

		return {.point = p, .rotation = r};
	}

	auto Transform::inv_multiply(const Transform& other) const noexcept -> Transform
	{
		const auto p = inv_transform(other.point);
		const auto r = rotation.inv_multiply(other.rotation);

		return {.point = p, .rotation = r};
	}
}
