// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
	/// The observed forces and torques are not returned for efficiency reasons.
	class JointThresholdExceededEvent final
	{
	public:
		/// The joint id
		JointId joint_id;
	};

	/// Joint events are buffered in the world and are available
	/// as event arrays after the time step is complete.
	/// Note: this data becomes invalid if joints are destroyed
	class JointEvents final
	{
	public:
		std::span<const JointThresholdExceededEvent> threshold_exceeded;
	};
}
