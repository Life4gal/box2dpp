// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// A line segment with two-sided collision.
	class Segment final
	{
	public:
		/// The first point
		Vec2 point1;

		/// The second point
		Vec2 point2;
	};

	/// A line segment with one-sided collision. Only collides on the right side.
	/// Several of these are generated for a chain shape.
	/// ghost1 -> point1 -> point2 -> ghost2
	class ChainSegment final
	{
	public:
		/// The tail ghost vertex
		Vec2 ghost1;

		/// The line segment
		Segment segment;

		/// The head ghost vertex
		Vec2 ghost2;

		/// The owning chain shape index (internal usage only)
		// id_type chain_id;
	};
}
