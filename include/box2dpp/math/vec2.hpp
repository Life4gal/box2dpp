// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <cmath>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	/// Check if a floating-point value is finite and not NaN
	[[nodiscard]] auto valid(float value) noexcept -> bool;

	/// 2D vector for points, directions, and coordinates
	class Vec2 final
	{
	public:
		const static Vec2 zero;

		float x;
		float y;

		// [[nodiscard]] constexpr auto operator<=>(const Vec2& other) const noexcept -> std::partial_ordering = default;
		[[nodiscard]] constexpr auto operator==(const Vec2& other) const noexcept -> bool = default;

		/// Check if vector components are finite and not NaN
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Check if vector is approximately unit length
		[[nodiscard]] auto normalized() const noexcept -> bool;

		/// Return unit vector in same direction, or zero if length is negligible
		[[nodiscard]] auto normalize() const noexcept -> Vec2;

		/// Return unit vector and also provide the original length
		[[nodiscard]] auto normalize(float& length) const noexcept -> Vec2;

		/// Dot product: a·b = a.x*b.x + a.y*b.y
		[[nodiscard]] auto dot(const Vec2& other) const noexcept -> float;

		/// 2D cross product (scalar): a×b = a.x*b.y - a.y*b.x
		/// Returns signed area of parallelogram, positive for counter-clockwise from a to b
		[[nodiscard]] auto cross(const Vec2& other) const noexcept -> float;

		/// Cross product with scalar (vector result): v×s = (v.y*s, -v.x*s)
		[[nodiscard]] auto cross(float scalar) const noexcept -> Vec2;

		/// Cross product with scalar (vector result): s×v = (-v.y*s, v.x*s)
		[[nodiscard]] friend auto cross(float scalar, const Vec2& vec2) noexcept -> Vec2;

		/// Left perpendicular (90° counter-clockwise rotation): (-y, x)
		[[nodiscard]] auto left_perpendicular() const noexcept -> Vec2;

		/// Right perpendicular (90° clockwise rotation): (y, -x)
		[[nodiscard]] auto right_perpendicular() const noexcept -> Vec2;

		/// Euclidean length (magnitude)
		[[nodiscard]] auto length() const noexcept -> float;

		/// Squared length (faster, avoids sqrt)
		[[nodiscard]] auto length_squared() const noexcept -> float;

		/// Distance between points
		[[nodiscard]] auto distance(const Vec2& other) const noexcept -> float;

		/// Squared distance between points
		[[nodiscard]] auto distance_squared(const Vec2& other) const noexcept -> float;

		/// Linear interpolation: (1-t)*this + t*other
		// ReSharper disable once IdentifierTypo
		[[nodiscard]] auto lerp(const Vec2& other, float t) const noexcept -> Vec2;

		/// Component-wise combination using a binary functor
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

		/// Component-wise combination using a compile-time binary function
		template<auto Functor>
		[[nodiscard]] constexpr auto combination(const Vec2& other) const noexcept -> Vec2 //
			requires requires
			{
				Functor(x, other.x);
				Functor(y, other.y);
			}
		{
			return {.x = Functor(x, other.x), .y = Functor(y, other.y)};
		}

		/// Component-wise minimum
		[[nodiscard]] auto combination_min(const Vec2& other) const noexcept -> Vec2;

		/// Component-wise maximum
		[[nodiscard]] auto combination_max(const Vec2& other) const noexcept -> Vec2;

		/// Component-wise absolute value
		[[nodiscard]] auto abs() const noexcept -> Vec2;

		/// Component-wise floor
		[[nodiscard]] auto floor() const noexcept -> Vec2;

		/// Component-wise ceiling
		[[nodiscard]] auto ceil() const noexcept -> Vec2;

		/// Reflect vector across a unit normal
		[[nodiscard]] auto reflect(const Vec2& normal) const noexcept -> Vec2;

		/// Project vector onto another vector
		[[nodiscard]] auto project(const Vec2& onto) const noexcept -> Vec2;

		/// Reject vector from another vector (perpendicular component)
		[[nodiscard]] auto reject(const Vec2& from) const noexcept -> Vec2;
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
		return vec2.abs();
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

	constexpr auto operator-=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs - rhs;
	}

	constexpr auto operator-=(Vec2& lhs, const float rhs) noexcept -> Vec2&
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

	constexpr auto operator+=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs + rhs;
	}

	constexpr auto operator+=(Vec2& lhs, const float rhs) noexcept -> Vec2&
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

	constexpr auto operator*=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs * rhs;
	}

	constexpr auto operator*=(Vec2& lhs, const float rhs) noexcept -> Vec2&
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

	constexpr auto operator/=(Vec2& lhs, const Vec2& rhs) noexcept -> Vec2&
	{
		return lhs = lhs / rhs;
	}

	constexpr auto operator/=(Vec2& lhs, const float rhs) noexcept -> Vec2&
	{
		return lhs /= Vec2{.x = rhs, .y = rhs};
	}

	// ==========================
	// TERNARY
	// ==========================

	/// Fused multiply-add: a + s * b
	[[nodiscard]] constexpr auto multiply_add(const Vec2& a, const float s, const Vec2& b) noexcept -> Vec2
	{
		return a + s * b;
	}

	/// Fused multiply-subtract: a - s * b
	[[nodiscard]] constexpr auto multiply_sub(const Vec2& a, const float s, const Vec2& b) noexcept -> Vec2
	{
		return a - s * b;
	}
} // namespace box2dpp
