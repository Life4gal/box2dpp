// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/chain.hpp>

namespace box2dpp
{
	namespace
	{
		const SurfaceMaterialDesc default_material{};
	}

	ChainDesc::ChainDesc() noexcept
		: BPP_DESC_USER_DATA_DEFINITION
		  materials{&default_material, 1},
		  is_loop{false},
		  enable_sensor_events{false} {}
}
