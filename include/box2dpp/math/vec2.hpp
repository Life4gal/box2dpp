// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <cmath>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	[[nodiscard]] auto valid(float value) noexcept -> bool;

	/// 2D vector
	/// This can be used to represent a point or free vector
	class Vec2 final
	{
	public:
		const static Vec2 zero;

		float x;
		float y;

		// [[nodiscard]] constexpr auto operator<=>(const Vec2& other) const noexcept -> std::partial_ordering = default;
		[[nodiscard]] constexpr auto operator==(const Vec2& other) const noexcept -> bool = default;

		[[nodiscard]] auto valid() const noexcept -> bool;

		[[nodiscard]] auto normalized() const noexcept -> bool;

		/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
		[[nodiscard]] auto normalize() const noexcept -> Vec2;

		/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
		/// Also outputs the length.
		[[nodiscard]] auto normalize(float& length) const noexcept -> Vec2;

		/// Vector dot product
		[[nodiscard]] auto dot(const Vec2& other) const noexcept -> float;

		/// Vector cross product.
		/// In 2D this yields a scalar.
		[[nodiscard]] auto cross(const Vec2& other) const noexcept -> float;

		/// Perform the cross product on a vector and a scalar.
		/// In 2D this produces a vector.
		[[nodiscard]] auto cross(float scalar) const noexcept -> Vec2;

		/// Get the length of this vector
		[[nodiscard]] auto length() const noexcept -> float;

		/// Get the length squared of this vector
		/// dot(self)
		[[nodiscard]] auto length_squared() const noexcept -> float;

		/// Get the distance between points
		[[nodiscard]] auto distance(const Vec2& other) const noexcept -> float;

		/// Get the distance squared between points
		[[nodiscard]] auto distance_squared(const Vec2& other) const noexcept -> float;

		/// Vector linear interpolation
		// ReSharper disable once IdentifierTypo
		[[nodiscard]] auto lerp(const Vec2& other, float t) const noexcept -> Vec2;

		template<typename Functor>
		[[nodiscard]] constexpr auto combination(const Vec2& other, Functor functor) const noexcept -> Vec2 //
			requires requires
			{
				functor(x, other.x);
				functor(y, other.y);
			}
		{
			return {.x = functor(x, other.x), .y = functor(y, other.y)};
		}

		[[nodiscard]] auto combination_min(const Vec2& other) const noexcept -> Vec2;

		[[nodiscard]] auto combination_max(const Vec2& other) const noexcept -> Vec2;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Vec2 Vec2::zero = {.x = 0, .y = 0};

	// ==========================
	// UNARY
	// ==========================

	[[nodiscard]] constexpr auto operator-(const Vec2& vec2) noexcept -> Vec2
	{
		return {.x = -vec2.x, .y = -vec2.y};
	}

	[[nodiscard]] /*constexpr*/ inline auto operator+(const Vec2& vec2) noexcept -> Vec2
	{
		return {.x = std::abs(vec2.x), .y = std::abs(vec2.y)};
	}

	// ==========================
	// BINARY
	// ==========================

	[[nodiscard]] constexpr auto operator-(const Vec2& lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return {.x = lhs.x - rhs.x, .y = lhs.y - rhs.y};
	}

	[[nodiscard]] constexpr auto operator-(const float lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return Vec2{.x = lhs, .y = lhs} - rhs;
	}

	[[nodiscard]] constexpr auto operator-(const Vec2& lhs, const float rhs) noexcept -> Vec2
	{
		return lhs - Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator-=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs - rhs;
	}

	[[nodiscard]] constexpr auto operator-=(Vec2& lhs, const float rhs) noexcept -> Vec2&
	{
		return lhs -= Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator+(const Vec2& lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return {.x = lhs.x + rhs.x, .y = lhs.y + rhs.y};
	}

	[[nodiscard]] constexpr auto operator+(const float lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return Vec2{.x = lhs, .y = lhs} + rhs;
	}

	[[nodiscard]] constexpr auto operator+(const Vec2& lhs, const float rhs) noexcept -> Vec2
	{
		return lhs + Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator+=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs + rhs;
	}

	[[nodiscard]] constexpr auto operator+=(Vec2& lhs, const float rhs) noexcept -> Vec2&
	{
		return lhs += Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator*(const Vec2& lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return {.x = lhs.x * rhs.x, .y = lhs.y * rhs.y};
	}

	[[nodiscard]] constexpr auto operator*(const float lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return Vec2{.x = lhs, .y = lhs} * rhs;
	}

	[[nodiscard]] constexpr auto operator*(const Vec2& lhs, const float rhs) noexcept -> Vec2
	{
		return lhs * Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator*=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs * rhs;
	}

	[[nodiscard]] constexpr auto operator*=(Vec2& lhs, const float rhs) noexcept -> Vec2&
	{
		return lhs *= Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator/(const Vec2& lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return {.x = lhs.x / rhs.x, .y = lhs.y / rhs.y};
	}

	[[nodiscard]] constexpr auto operator/(const float lhs, const Vec2& rhs) noexcept -> Vec2
	{
		return Vec2{.x = lhs, .y = lhs} / rhs;
	}

	[[nodiscard]] constexpr auto operator/(const Vec2& lhs, const float rhs) noexcept -> Vec2
	{
		return lhs / Vec2{.x = rhs, .y = rhs};
	}

	[[nodiscard]] constexpr auto operator/=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs / rhs;
	}

	[[nodiscard]] constexpr auto operator/=(Vec2& lhs, const float rhs) noexcept -> Vec2&
	{
		return lhs /= Vec2{.x = rhs, .y = rhs};
	}
} // namespace box2dpp
