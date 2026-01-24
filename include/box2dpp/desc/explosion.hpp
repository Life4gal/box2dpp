// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

namespace box2dpp
{
	/// The explosion definition is used to configure options for explosions. 
	/// Explosions consider shape geometry when computing the impulse.
	class ExplosionDesc final
	{
	public:
		/// Mask bits to filter shapes
		/// Only shapes with matching category bits will be affected by the explosion
		BPP_DESC_FILTER_CATEGORY_TYPE mask;

		/// The center of the explosion in world space
		/// Position from which explosion forces radiate outward
		Vec2 position;

		/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
		/// Defines the smooth transition zone where impulse linearly decreases from full to zero
		/// @note If falloff is 0, impulse stops abruptly at the radius
		float falloff;

		/// Impulse per unit length. 
		/// This applies an impulse according to the shape perimeter that is facing the explosion. 
		/// Explosions only apply to circles, capsules, and polygons. 
		/// This may be negative for implosions.
		/// @note The total impulse is proportional to the exposed perimeter facing the explosion center
		float impulse_per_length;

		ExplosionDesc() noexcept;
	};
}
