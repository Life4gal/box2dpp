// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// Convert any angle into the range [-pi, pi]
	[[nodiscard]] auto unwind_angle(float radians) noexcept -> float;

	/// 2D rotation
	/// This is similar to using a complex number for rotation
	class Rotation final
	{
	public:
		const static Rotation identity;

		float cos;
		float sin;

		/// Make a rotation using an angle in radians
		[[nodiscard]] static auto from(float radians) noexcept -> Rotation;

		/// Make a rotation using a unit vector
		[[nodiscard]] static auto from(const Vec2& unit_vector) noexcept -> Rotation;

		/// Compute the rotation between two unit vectors
		[[nodiscard]] static auto from(const Vec2& unit_vector1, const Vec2& unit_vector2) noexcept -> Rotation;

	private:
		[[nodiscard]] auto valid_angle() const noexcept -> bool;

	public:
		[[nodiscard]] auto valid() const noexcept -> bool;

		[[nodiscard]] auto normalized() const noexcept -> bool;

		/// Normalize rotation
		[[nodiscard]] auto normalize() const noexcept -> Rotation;

		/// Integrate rotation from angular velocity
		/// @param delta the angular displacement in radians
		[[nodiscard]] auto integrate(float delta) const noexcept -> Rotation;

		/// Get the angle in radians in the range [-pi, pi]
		[[nodiscard]] auto angle() const noexcept -> float;

		/// Relative angle between two rotation
		[[nodiscard]] auto angle(const Rotation& other) const noexcept -> float;

		/// Get the x-axis
		[[nodiscard]] auto axis_x() const noexcept -> Vec2;

		/// Get the y-axis
		[[nodiscard]] auto axis_y() const noexcept -> Vec2;

		/// Normalized linear interpolation
		// ReSharper disable once IdentifierTypo
		[[nodiscard]] auto nlerp(const Rotation& other, float t) const noexcept -> Rotation;

		// Inverse a rotation
		[[nodiscard]] auto inv() const noexcept -> Rotation;

		/// Rotate a vector
		[[nodiscard]] auto rotate(const Vec2& vec2) const noexcept -> Vec2;

		/// Inverse rotate a vector
		[[nodiscard]] auto inv_rotate(const Vec2& vec2) const noexcept -> Vec2;

		/// Multiply two rotations
		[[nodiscard]] auto multiply(const Rotation& other) const noexcept -> Rotation;

		/// Transpose multiply two rotations
		[[nodiscard]] auto inv_multiply(const Rotation& other) const noexcept -> Rotation;
	};


	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Rotation Rotation::identity = {.cos = 1.f, .sin = 0.f};
}
