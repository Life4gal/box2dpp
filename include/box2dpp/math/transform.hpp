// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/rotation.hpp>

namespace box2dpp
{
	/// A 2D rigid transform
	class Transform final
	{
	public:
		const static Transform identity;

		Vec2 point;
		Rotation rotation;

		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Transform a point (e.g. local space to world space)
		[[nodiscard]] auto transform(const Vec2& p) const noexcept -> Vec2;

		/// Inverse transform a point (e.g. world space to local space)
		[[nodiscard]] auto inv_transform(const Vec2& p) const noexcept -> Vec2;

		/// Multiply two transforms
		/// If the result is applied to a point p local to frame B,
		/// the transform would first convert p to a point local to frame A,
		/// then into a point in the world frame.
		[[nodiscard]] auto multiply(const Transform& other) const noexcept -> Transform;

		/// Creates a transform that converts a local point in frame B to a local point in frame A.
		[[nodiscard]] auto inv_multiply(const Transform& other) const noexcept -> Transform;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Transform Transform::identity{.point = Vec2::zero, .rotation = Rotation::identity};
}
