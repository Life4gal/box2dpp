// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <concepts>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	template<std::integral To, std::integral From>
	[[nodiscard]] constexpr auto value_cast(const From value) noexcept -> To
	{
		constexpr auto st = std::is_signed_v<To>;
		constexpr auto sf = std::is_signed_v<From>;

		if constexpr (std::is_same_v<To, From>)
		{
			return value;
		}
		// else if constexpr (sizeof(To) > sizeof(From))
		// {
		// 	if constexpr (st and sf)
		// 	{
		// 		// signed ==> signed
		// 		return static_cast<To>(value);
		// 	}
		// 	else if constexpr (not st and not sf)
		// 	{
		// 		// unsigned ==> unsigned
		// 		return static_cast<To>(value);
		// 	}
		// 	else if constexpr (st)
		// 	{
		// 		// unsigned ==> signed
		// 		BPP_ASSERT(static_cast<std::uintmax_t>(value) <= static_cast<std::uintmax_t>(std::numeric_limits<To>::max()));
		// 		return static_cast<To>(value);
		// 	}
		// 	else
		// 	{
		// 		// signed ==> unsigned
		// 		BPP_ASSERT(value >= 0);
		// 		BPP_ASSERT(static_cast<std::uintmax_t>(value) <= static_cast<std::uintmax_t>(std::numeric_limits<To>::max()));
		// 		return static_cast<To>(value);
		// 	}
		// }
		// else
		// {
		// 	if constexpr (st and sf)
		// 	{
		// 		// signed ==> signed
		// 		BPP_ASSERT(value >= std::numeric_limits<To>::min());
		// 		BPP_ASSERT(value <= std::numeric_limits<To>::max());
		// 		return static_cast<To>(value);
		// 	}
		// 	else if constexpr (not st and not sf)
		// 	{
		// 		// unsigned ==> unsigned
		// 		BPP_ASSERT(value <= std::numeric_limits<To>::max());
		// 		return static_cast<To>(value);
		// 	}
		// 	else if constexpr (st)
		// 	{
		// 		// unsigned ==> signed
		// 		BPP_ASSERT(value <= static_cast<std::make_unsigned_t<To>>(std::numeric_limits<To>::max()));
		// 		return static_cast<To>(value);
		// 	}
		// 	else
		// 	{
		// 		// signed ==> unsigned
		// 		BPP_ASSERT(value >= 0);
		// 		BPP_ASSERT(static_cast<std::make_unsigned_t<From>>(value) <= std::numeric_limits<To>::max());
		// 		return static_cast<To>(value);
		// 	}
		// }
		else
		{
			BPP_ASSERT(std::in_range<To>(value));
			return static_cast<To>(value);
		}
	}
}
