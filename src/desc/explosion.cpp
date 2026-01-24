// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/explosion.hpp>

#include <limits>

namespace box2dpp
{
	ExplosionDesc::ExplosionDesc() noexcept
		: mask{BPP_DESC_FILTER_DEFAULT_CATEGORY_MASK},
		  position{Vec2::zero},
		  falloff{0},
		  impulse_per_length{0} {}
}
