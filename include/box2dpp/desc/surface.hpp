// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/version.hpp>

namespace box2dpp
{
	/// Surface materials allow chain shapes to have per segment surface properties.
	class SurfaceMaterialDesc final
	{
	public:
		/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
		/// Determines how much force resists sliding motion
		/// 0 = no friction (ice), 1 = high friction (rubber)
		float friction;

		/// The coefficient of restitution (bounce) usually in the range [0,1].
		/// Determines how much energy is conserved in collisions
		/// 0 = perfectly inelastic (no bounce), 1 = perfectly elastic (full bounce)
		/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
		float restitution;

		/// The rolling resistance usually in the range [0,1].
		/// Determines how much force resists rolling motion
		/// Higher values cause objects to stop rolling more quickly
		float rolling_resistance;

		/// The tangent speed for conveyor belts
		/// Surface velocity in meters per second along the tangent direction
		/// Positive values move objects to the right of the surface normal
		float tangent_speed;

		/// User material identifier. 
		/// This is passed with query results and to friction and restitution combining functions. 
		/// It is not used internally.
		BPP_DESC_SURFACE_MATERIAL_ID_TYPE id;

		SurfaceMaterialDesc() noexcept;
	};
}
