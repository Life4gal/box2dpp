// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// World definition used to create a simulation world.
	class WorldDesc final
	{
	public:
		/// Optional friction mixing callback. 
		/// This intentionally provides no context objects because this is called from a worker thread.
		/// @warning This function should not attempt to modify Box2D state or user application state.
		using friction_callback_type = auto (*)(
			float friction_a,
			BPP_DESC_SURFACE_MATERIAL_ID_TYPE material_id_a,
			float friction_b,
			BPP_DESC_SURFACE_MATERIAL_ID_TYPE material_id_b
		) noexcept -> float;

		/// Optional restitution mixing callback. 
		/// This intentionally provides no context objects because this is called from a worker thread.
		/// @warning This function should not attempt to modify Box2D state or user application state.
		using restitution_callback_type = auto (*)(
			float restitution_a,
			BPP_DESC_SURFACE_MATERIAL_ID_TYPE material_id_a,
			float restitution_b,
			BPP_DESC_SURFACE_MATERIAL_ID_TYPE material_id_b
		) noexcept -> float;

		// todo: Task Callback
		BPP_DESC_TASK_CONTEXT_DECLARATION

		BPP_DESC_USER_DATA_DECLARATION

		/// Gravity vector. 
		Vec2 gravity;

		/// Restitution speed threshold, usually in m/s. 
		/// Collisions above this speed have restitution applied (will bounce).
		float restitution_threshold;

		/// Threshold speed for hit events. 
		/// Usually meters per second.
		float hit_event_threshold;

		/// Contact stiffness. 
		/// Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
		float contact_hertz;

		/// Contact bounciness. Non-dimensional. 
		/// You can speed up overlap recovery by decreasing this with the trade-off that overlap resolution becomes more energetic.
		float contact_damping_ratio;

		/// This parameter controls how fast overlap is resolved and usually has units of meters per second. 
		/// This only puts a cap on the resolution speed. 
		/// The resolution speed is increased by increasing the hertz and/or decreasing the damping ratio.
		float contact_speed;

		/// Maximum linear speed. Usually meters per second.
		float maximum_linear_speed;

		/// Optional mixing callback for friction. 
		/// The default uses sqrt(friction_a * friction_b).
		friction_callback_type friction_callback;

		/// Optional mixing callback for restitution. 
		/// The default uses max(restitution_a, restitution_b).
		restitution_callback_type restitution_callback;

		/// Can bodies go to sleep to improve performance
		bool enable_sleep;

		/// Enable continuous collision
		bool enable_continuous;

		/// Contact softening when mass ratios are large. Experimental.
		bool enable_contact_softening;

		WorldDesc() noexcept;
	};
}
