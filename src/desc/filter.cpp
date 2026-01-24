// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/filter.hpp>

#include <limits>

namespace box2dpp
{
	FilterDesc::FilterDesc() noexcept
		: category{BPP_DESC_FILTER_DEFAULT_CATEGORY},
		  mask{BPP_DESC_FILTER_DEFAULT_CATEGORY_MASK},
		  index{invalid_index} {}

	QueryFilterDesc::QueryFilterDesc() noexcept
		: category{BPP_DESC_FILTER_DEFAULT_CATEGORY},
		  mask{BPP_DESC_FILTER_DEFAULT_CATEGORY_MASK} {}
}
