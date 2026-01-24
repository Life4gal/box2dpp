// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math/transform.hpp>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// Body move events triggered when a body moves.
	/// Triggered when a body moves due to simulation. 
	/// Not reported for bodies moved by the user.
	/// This also has a flag to indicate that the body went to sleep 
	/// so the application can also sleep that actor/entity/object associated with the body.
	/// On the other hand if the flag does not indicate the body went to sleep 
	/// then the application can treat the actor/entity/object associated with the body as awake.
	/// This is an efficient way for an application to update game object transforms 
	/// rather than calling functions such as ```Body::get_transform()```  <-- todo: interface design
	/// because this data is delivered as a contiguous array, 
	/// and it is only populated with bodies that have moved.
	/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
	class BodyMoveEvent final // NOLINT(clang-diagnostic-padded)
	{
	public:
		Transform transform;

		BodyId body_id;

		bool fell_asleep;
	};

	/// Body events are buffered in the world and are available
	/// as event arrays after the time step is complete.
	/// Note: this data becomes invalid if bodies are destroyed
	class BodyEvents final
	{
	public:
		std::span<const BodyMoveEvent> moves;
	};
}
