// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/world.hpp>

namespace box2dpp
{
	WorldDesc::WorldDesc() noexcept
		: BPP_DESC_TASK_CONTEXT_DEFINITION
		  BPP_DESC_USER_DATA_DEFINITION
		  gravity{.x = 0, .y = -9.8f},
		  restitution_threshold{1 * units_count_per_meter},
		  hit_event_threshold{1 * units_count_per_meter},
		  contact_hertz{30},
		  contact_damping_ratio{10},
		  contact_speed{3.f * units_count_per_meter},
		  // 400 meters per second, faster than the speed of sound
		  maximum_linear_speed{400.f * units_count_per_meter},
		  friction_callback{nullptr},
		  restitution_callback{nullptr},
		  enable_sleep{true},
		  enable_continuous{true},
		  enable_contact_softening{false} {}
}
