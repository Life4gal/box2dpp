// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/body.hpp>

namespace box2dpp
{
	BodyDesc::BodyDesc() noexcept
		: type{BodyType::STATIC},
		  position{Vec2::zero},
		  rotation{Rotation::identity},
		  linear_velocity{Vec2::zero},
		  angular_velocity{0},
		  linear_damping{0},
		  angular_damping{0},
		  gravity_scale{1},
		  sleep_threshold{.05f * units_count_per_meter},
		  motion_locks{MotionLocks::NONE},
		  enable_sleep{true},
		  is_awake{true},
		  is_bullet{false},
		  is_enabled{true},
		  allow_fast_rotation{false} {}
}
