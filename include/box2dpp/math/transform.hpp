// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/rotation.hpp>

namespace box2dpp
{
	/// A 2D rigid transform representing position and orientation
	/// Stored as separate translation and rotation for efficiency
	class Transform final
	{
	public:
		const static Transform identity;

		/// Translation component (world position)
		Vec2 point;
		/// Rotation component (orientation)
		Rotation rotation;

		[[nodiscard]] constexpr auto operator==(const Transform& other) const noexcept -> bool = default;

		/// Check if transform components are mathematically valid
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Transform point from local to world coordinates: p' = R * p + t
		[[nodiscard]] auto transform(const Vec2& p) const noexcept -> Vec2;

		/// Inverse transform point from world to local coordinates: p' = R⁻¹ * (p - t)
		[[nodiscard]] auto inv_transform(const Vec2& p) const noexcept -> Vec2;

		/// Transform vector (ignoring translation): v' = R * v
		[[nodiscard]] auto transform_vector(const Vec2& v) const noexcept -> Vec2;

		/// Inverse transform vector (ignoring translation): v' = R⁻¹ * v
		[[nodiscard]] auto inv_transform_vector(const Vec2& v) const noexcept -> Vec2;

		/// Compose transforms: returns this × other (apply other first, then this)
		/// If other is in local frame B, result transforms from B to world via A
		[[nodiscard]] auto multiply(const Transform& other) const noexcept -> Transform;

		/// Multiply by inverse: returns this⁻¹ × other (relative transform from this to other)
		/// Transforms points from other's local space to this local space
		[[nodiscard]] auto multiply_by_inv(const Transform& other) const noexcept -> Transform;

		/// Inverse multiply: returns other × this⁻¹ (change of coordinate frames)
		/// Transforms points from this local space to other local space
		[[nodiscard]] auto inv_multiply(const Transform& other) const noexcept -> Transform;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Transform Transform::identity{.point = Vec2::zero, .rotation = Rotation::identity};
}
