// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/desc/joint.hpp>

#include <limits>

namespace box2dpp
{
	namespace joint_detail
	{
		Base::Base() noexcept
			:
			// body_a{},
			// body_b{},
			local_frame_a{Transform::identity},
			local_frame_b{Transform::identity},
			force_threshold{std::numeric_limits<float>::max()},
			torque_threshold{std::numeric_limits<float>::max()},
			constraint_hertz{60},
			constraint_damping_ratio{2},
			draw_scale{units_count_per_meter},
			collide_connected{false} {}
	}

	DistanceJointDesc::DistanceJointDesc() noexcept
		:
		length{1},
		enable_spring{false},
		lower_spring_force{std::numeric_limits<float>::lowest()},
		upper_spring_force{std::numeric_limits<float>::max()},
		hertz{0},
		damping_ratio{0},
		enable_limit{false},
		min_length{0},
		max_length{BPP_HUGE},
		enable_motor{false},
		max_motor_force{0},
		motor_speed{0} {}

	MotorJointDesc::MotorJointDesc() noexcept
		:
		linear_velocity{Vec2::zero},
		max_velocity_force{0},
		angular_velocity{0},
		max_velocity_torque{0},
		linear_hertz{0},
		linear_damping_ratio{0},
		max_spring_force{0},
		angular_hertz{0},
		angular_damping_ratio{0},
		max_spring_torque{0} {}

	FilterJointDesc::FilterJointDesc() noexcept = default;

	PrismaticJointDesc::PrismaticJointDesc() noexcept
		:
		enable_spring{false},
		hertz{0},
		damping_ratio{0},
		target_translation{0},
		enable_limit{false},
		lower_translation{0},
		upper_translation{0},
		enable_motor{false},
		max_motor_force{0},
		motor_speed{0} {}

	RevoluteJointDesc::RevoluteJointDesc() noexcept
		:
		target_angle{0},
		enable_spring{false},
		hertz{0},
		damping_ratio{0},
		enable_limit{false},
		lower_angle{0},
		upper_angle{0},
		enable_motor{false},
		max_motor_torque{0},
		motor_speed{0} {}

	WeldJointDesc::WeldJointDesc() noexcept
		:
		linear_hertz{0},
		angular_hertz{0},
		linear_damping_ratio{0},
		angular_damping_ratio{0} {}

	WheelJointDesc::WheelJointDesc() noexcept
		:
		enable_spring{true},
		hertz{1},
		damping_ratio{.7f},
		enable_limit{false},
		lower_translation{0},
		upper_translation{0},
		enable_motor{false},
		max_motor_torque{0},
		motor_speed{0} {}
}
