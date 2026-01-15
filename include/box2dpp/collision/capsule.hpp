// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A solid capsule can be viewed as two semicircles connected by a rectangle.
	class Capsule final
	{
	public:
		/// Local center of the first semicircle
		Vec2 center1;

		/// Local center of the second semicircle
		Vec2 center2;

		/// The radius of the semicircles
		float radius;

		/// Test a point for overlap with a capsule in local space
		[[nodiscard]] auto in(Vec2 point) const noexcept -> bool;
	};
}
