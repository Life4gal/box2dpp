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

	/// 2D rotation represented as a unit complex number (cos(θ), sin(θ))
	/// This is more efficient than storing angles and recomputing trig functions
	class Rotation final
	{
	public:
		const static Rotation identity;

		/// Cosine component of the rotation (real part)
		float cos;
		/// Sine component of the rotation (imaginary part)
		float sin;

		/// Construct a rotation from an angle in radians
		/// Uses rational approximations to avoid expensive std::cos/sin calls
		[[nodiscard]] static auto from(float radians) noexcept -> Rotation;

		/// Construct a rotation from a unit vector representing (cos(θ), sin(θ))
		[[nodiscard]] static auto from(const Vec2& unit_vector) noexcept -> Rotation;

		/// Construct rotation that rotates unit_vector1 to align with unit_vector2
		/// Returns r such that r.rotate(unit_vector1) ≈ unit_vector2
		[[nodiscard]] static auto from(const Vec2& unit_vector1, const Vec2& unit_vector2) noexcept -> Rotation;

	private:
		/// Check if individual components are valid floating-point numbers
		[[nodiscard]] auto valid_angle() const noexcept -> bool;

	public:
		/// Check if rotation is mathematically valid (finite and normalized)
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Check if rotation is approximately normalized (unit length)
		[[nodiscard]] auto normalized() const noexcept -> bool;

		/// Normalize to unit length, returns identity if length is zero
		[[nodiscard]] auto normalize() const noexcept -> Rotation;

		/// Apply angular displacement to this rotation
		/// @param delta Angular displacement in radians (positive for counter-clockwise)
		[[nodiscard]] auto integrate(float delta) const noexcept -> Rotation;

		/// Extract angle in radians in the principal range [-pi, pi]
		[[nodiscard]] auto angle() const noexcept -> float;

		/// Relative angle from this rotation to another: angle(this⁻¹ × other)
		/// Returns signed smallest angle between two rotations
		[[nodiscard]] auto angle(const Rotation& other) const noexcept -> float;

		/// Get the x-axis (local right direction) after applying this rotation
		[[nodiscard]] auto axis_x() const noexcept -> Vec2;

		/// Get the y-axis (local up direction) after applying this rotation
		[[nodiscard]] auto axis_y() const noexcept -> Vec2;

		/// Normalized linear interpolation (faster than SLERP for 2D)
		/// Interpolates between unit complex numbers and renormalizes
		// ReSharper disable once IdentifierTypo
		[[nodiscard]] auto nlerp(const Rotation& other, float t) const noexcept -> Rotation;

		/// Get the inverse/conjugate rotation: (cos(θ), -sin(θ))
		[[nodiscard]] auto inv() const noexcept -> Rotation;

		/// Rotate a vector by this rotation: v' = R * v
		[[nodiscard]] auto rotate(const Vec2& vec2) const noexcept -> Vec2;

		/// Inverse rotate a vector: v' = R⁻¹ * v = Rᵀ * v
		[[nodiscard]] auto inv_rotate(const Vec2& vec2) const noexcept -> Vec2;

		/// Compose rotations: returns this × other (apply other first, then this)
		[[nodiscard]] auto multiply(const Rotation& other) const noexcept -> Rotation;

		/// Multiply by inverse: returns this⁻¹ × other (relative rotation from this to other)
		/// Represents rotation needed to go from this orientation to other orientation
		[[nodiscard]] auto multiply_by_inv(const Rotation& other) const noexcept -> Rotation;

		/// Inverse multiply: returns other × this⁻¹ (change of basis from this frame to other frame)
		/// Represents rotation from other's perspective of this rotation
		[[nodiscard]] auto inv_multiply(const Rotation& other) const noexcept -> Rotation;
	};


	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Rotation Rotation::identity = {.cos = 1.f, .sin = 0.f};
}
