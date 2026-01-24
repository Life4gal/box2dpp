// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/version.hpp>

namespace box2dpp
{
	/// This is used to filter collision on shapes. 
	/// It affects shape-vs-shape collision and shape-versus-query collision (such as World::cast_ray).
	class FilterDesc final
	{
	public:
		using index_type = std::int32_t;

		constexpr static index_type invalid_index = 0;

		/// The collision category bits. 
		/// Normally you would just set one bit. 
		/// The category bits should represent your application object types. 
		/// For example:
		/// @code{.cpp}
		/// enum MyCategories
		/// {
		///    Static  = 0x00000001,
		///    Dynamic = 0x00000002,
		///    Debris  = 0x00000004,
		///    Player  = 0x00000008,
		///    // etc.
		/// };
		/// @endcode
		BPP_DESC_FILTER_CATEGORY_TYPE category;

		/// The collision mask bits. 
		/// This states the categories that this shape would accept for collision.
		/// For example, you may want your player to only collide with static objects and other players.
		/// @code{.c}
		/// maskBits = Static | Player;
		/// @endcode
		/// @note Collision occurs if (shapeA.mask & shapeB.category) != 0 AND (shapeB.mask & shapeA.category) != 0
		BPP_DESC_FILTER_CATEGORY_TYPE mask;

		/// Collision groups allow a certain group of objects to never collide (negative) or always collide (positive). 
		/// A group index of zero(invalid_index) has no effect. 
		/// Non-zero group filtering always wins against the mask bits.
		/// For example, you may want rag-dolls to collide with other rag-dolls, but you don't want rag-doll self-collision. 
		/// In this case you would give each rag-doll a unique negative group index and apply that group index to all shapes on the rag-doll.
		index_type index;

		FilterDesc() noexcept;
	};

	/// The query filter is used to filter collisions between queries and shapes. 
	/// For example, you may want a ray-cast representing a projectile to hit players 
	/// and the static environment but not debris.
	class QueryFilterDesc final
	{
	public:
		/// The collision category bits of this query. Normally you would just set one bit.
		BPP_DESC_FILTER_CATEGORY_TYPE category;

		/// The collision mask bits. This states the shape categories that this query would accept for collision.
		BPP_DESC_FILTER_CATEGORY_TYPE mask;

		QueryFilterDesc() noexcept;
	};
}
