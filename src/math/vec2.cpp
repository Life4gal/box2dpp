// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/math/vec2.hpp>

#include <cmath>
#include <limits>
#include <ranges>

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

		// Check if length squared is approximately 1 within tolerance
		return std::abs(1.f - l2) < std::numeric_limits<float>::epsilon() * 100.f;
	}

	auto Vec2::normalize() const noexcept -> Vec2
	{
		float length;
		return normalize(length);
	}

	auto Vec2::normalize(float& length) const noexcept -> Vec2
	{
		length = this->length();

		// Handle tiny vectors to avoid division by near-zero
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
		// 2D cross product returns scalar representing signed area
		return x * other.y - y * other.x;
	}

	auto Vec2::cross(const float scalar) const noexcept -> Vec2
	{
		// v × s = (v.y * s, -v.x * s) - 90° clockwise rotation scaled by s
		return {.x = y * scalar, .y = -x * scalar};
	}

	auto cross(const float scalar, const Vec2& vec2) noexcept -> Vec2
	{
		// s × v = (-v.y * s, v.x * s) - 90° counter-clockwise rotation scaled by s
		return vec2.cross(-scalar);
	}

	auto Vec2::left_perpendicular() const noexcept -> Vec2
	{
		// 90° counter-clockwise rotation: (-y, x)
		using box2dpp::cross;
		return cross(1.f, *this);
	}

	auto Vec2::right_perpendicular() const noexcept -> Vec2
	{
		// 90° clockwise rotation: (y, -x)
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
		// Linear interpolation: (1-t)*this + t*other
		return {.x = (1.f - t) * x + t * other.x, .y = (1.f - t) * y + t * other.y};
	}

	auto Vec2::combination_min(const Vec2& other) const noexcept -> Vec2
	{
		// Component-wise minimum
		// return combination(other, std::ranges::min);
		return combination<std::ranges::min>(other);
	}

	auto Vec2::combination_max(const Vec2& other) const noexcept -> Vec2
	{
		// Component-wise maximum
		// return combination(other, std::ranges::max);
		return combination<std::ranges::max>(other);
	}

	auto Vec2::abs() const noexcept -> Vec2
	{
		return {.x = std::abs(x), .y = std::abs(y)};
	}

	auto Vec2::floor() const noexcept -> Vec2
	{
		return {.x = std::floor(x), .y = std::floor(y)};
	}

	auto Vec2::ceil() const noexcept -> Vec2
	{
		return {.x = std::ceil(x), .y = std::ceil(y)};
	}

	auto Vec2::reflect(const Vec2& normal) const noexcept -> Vec2
	{
		// Reflection formula: v' = v - 2*(v·n)*n
		BPP_ASSERT(normal.normalized());

		const auto dot_prod = dot(normal);
		return *this - 2.f * dot_prod * normal;
	}

	auto Vec2::project(const Vec2& onto) const noexcept -> Vec2
	{
		// Projection of v onto u: (v·u / u·u) * u
		const auto denominator = onto.length_squared();
		if (denominator < std::numeric_limits<float>::epsilon())
		{
			return zero;
		}
		return (dot(onto) / denominator) * onto;
	}

	auto Vec2::reject(const Vec2& from) const noexcept -> Vec2
	{
		// Rejection (perpendicular component): v - proj_u(v)
		return *this - project(from);
	}
}
