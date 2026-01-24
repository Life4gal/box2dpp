// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/surface.hpp>

namespace box2dpp
{
	SurfaceMaterialDesc::SurfaceMaterialDesc() noexcept
		: friction{.6f},
		  restitution{0},
		  rolling_resistance{0},
		  tangent_speed{0},
		  id{BPP_DESC_DEFAULT_SURFACE_MATERIAL_ID} {}
}
