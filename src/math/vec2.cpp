// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/math/vec2.hpp>

#include <cmath>
#include <limits>
#include <ranges>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	auto valid(const float value) noexcept -> bool
	{
		return not(std::isnan(value) or std::isinf(value));
	}

	auto Vec2::valid() const noexcept -> bool
	{
		return box2dpp::valid(x) and box2dpp::valid(y);
	}

	auto Vec2::normalized() const noexcept -> bool
	{
		const auto l2 = length_squared();

		return std::abs(1.f - l2) < std::numeric_limits<float>::epsilon() * 100.f;
	}

	auto Vec2::normalize() const noexcept -> Vec2
	{
		float length = 0;
		return normalize(length);
	}

	auto Vec2::normalize(float& length) const noexcept -> Vec2
	{
		length = this->length();
		if (length < std::numeric_limits<float>::epsilon())
		{
			return {.x = 0, .y = 0};
		}

		const auto inv_length = 1.f / length;
		return {.x = x * inv_length, .y = y * inv_length};
	}

	auto Vec2::dot(const Vec2& other) const noexcept -> float
	{
		return x * other.x + y * other.y;
	}

	auto Vec2::cross(const Vec2& other) const noexcept -> float
	{
		return x * other.y - y * other.x;
	}

	auto Vec2::cross(const float scalar) const noexcept -> Vec2
	{
		return {.x = y * scalar, .y = -x * scalar};
	}

	auto cross(const float scalar, const Vec2& vec2) noexcept -> Vec2
	{
		return vec2.cross(-scalar);
	}

	auto Vec2::left_perpendicular() const noexcept -> Vec2
	{
		// force adl
		return box2dpp::cross(1.f, *this);
	}

	auto Vec2::right_perpendicular() const noexcept -> Vec2
	{
		return cross(1.f);
	}

	auto Vec2::length() const noexcept -> float
	{
		return std::sqrt(length_squared());
	}

	auto Vec2::length_squared() const noexcept -> float
	{
		return dot(*this);
	}

	auto Vec2::distance(const Vec2& other) const noexcept -> float
	{
		const auto diff = other - *this;
		return diff.length();
	}

	auto Vec2::distance_squared(const Vec2& other) const noexcept -> float
	{
		const auto diff = other - *this;
		return diff.length_squared();
	}

	// ReSharper disable once IdentifierTypo
	auto Vec2::lerp(const Vec2& other, const float t) const noexcept -> Vec2
	{
		return {.x = (1.f - t) * x + t * other.x, .y = (1.f - t) * y + t * other.y};
	}

	auto Vec2::combination_min(const Vec2& other) const noexcept -> Vec2
	{
		// return combination(other, std::ranges::min);
		return combination<std::ranges::min>(other);
	}

	auto Vec2::combination_max(const Vec2& other) const noexcept -> Vec2
	{
		// return combination(other, std::ranges::max);
		return combination<std::ranges::max>(other);
	}
}
