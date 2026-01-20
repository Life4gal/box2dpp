// This file is part of box2dpp
// Copyright (C) 2022-2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape.hpp>

namespace box2dpp
{
	ShapeDesc::ShapeDesc() noexcept
		:
		density{1.f},
		enable_custom_filtering{false},
		is_sensor{false},
		enable_sensor_events{false},
		enable_contact_events{false},
		enable_hit_events{false},
		enable_pre_solve_events{false},
		invoke_contact_creation{true},
		update_body_mass{true} {}
}
