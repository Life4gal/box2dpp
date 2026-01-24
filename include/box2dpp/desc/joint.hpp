// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/transform.hpp>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// Joint type enumeration
	/// This is useful because all joint types use JointId , 
	/// and sometimes you want to get the type of  joint.
	/// @ingroup joint
	enum class JointType : std::uint8_t
	{
		DISTANCE,
		FILTER,
		MOTOR,
		PRISMATIC,
		REVOLUTE,
		WELD,
		WHEEL,
	};

	namespace joint_detail
	{
		/// Base joint definition used by all joint types.
		/// The local frames are measured from the body's origin rather than the center of mass because:
		/// 1. you might not know where the center of mass will be
		/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
		class Base
		{
		public:
			/// The first attached body.
			/// Body to which the joint's local frame A is attached
			BodyId body_a;

			/// The second attached body.
			/// Body to which the joint's local frame B is attached
			BodyId body_b;

			/// The first local joint frame.
			/// Transform from body A's origin to the joint anchor/axis in A's local space
			Transform local_frame_a;

			/// The second local joint frame.
			/// Transform from body B's origin to the joint anchor/axis in B's local space
			Transform local_frame_b;

			/// Force threshold for joint events
			/// Minimum force magnitude required to trigger joint break/force events
			float force_threshold;

			/// Torque threshold for joint events
			/// Minimum torque magnitude required to trigger joint break/torque events
			float torque_threshold;

			/// Constraint hertz (advanced feature)
			/// Frequency of the constraint solver in Hertz (cycles per second)
			/// Higher values make constraints stiffer but may cause instability
			float constraint_hertz;

			/// Constraint damping ratio (advanced feature)
			float constraint_damping_ratio;

			/// Debug draw scale
			/// Scaling factor for visualizing the joint in debug rendering
			/// Typically set to match world units per meter
			float draw_scale;

			/// Set this flag to true if the attached bodies should collide
			/// When false, bodies connected by this joint will not collide with each other
			/// Useful for creating composite objects from multiple bodies
			bool collide_connected;

			Base() noexcept;
		};
	}

	/// Distance joint definition
	/// Connects a point on body A with a point on body B by a segment.
	/// Useful for ropes and springs.
	class DistanceJointDesc final : public joint_detail::Base
	{
	public:
		/// The rest length of this joint. Clamped to a stable minimum value.
		float length;

		/// Enable the distance constraint to behave like a spring. 
		/// If false then the distance joint will be rigid, overriding the limit and motor.
		bool enable_spring;

		/// The lower spring force controls how much tension it can sustain.
		float lower_spring_force;

		/// The upper spring force controls how much compression it a sustain.
		float upper_spring_force;

		/// The spring linear stiffness Hertz, cycles per second.
		float hertz;

		/// The spring linear damping ratio, non-dimensional.
		float damping_ratio;

		/// Enable/disable the joint limit
		bool enable_limit;

		/// Minimum length. Clamped to a stable minimum value.
		float min_length;

		/// Maximum length. Must be greater than or equal to the minimum length.
		float max_length;

		/// Enable/disable the joint motor.
		bool enable_motor;

		/// The maximum motor force, usually in newtons.
		float max_motor_force;

		/// The desired motor speed, usually in meters per second.
		float motor_speed;

		DistanceJointDesc() noexcept;
	};

	/// A motor joint is used to control the relative velocity and or transform between two bodies.
	/// With a velocity of zero this acts like top-down friction.
	class MotorJointDesc final : public joint_detail::Base
	{
	public:
		/// The desired linear velocity
		Vec2 linear_velocity;

		/// The maximum motor force in newtons
		float max_velocity_force;

		/// The desired angular velocity
		float angular_velocity;

		/// The maximum motor torque in newton-meters
		float max_velocity_torque;

		/// Linear spring hertz for position control
		float linear_hertz;

		/// Linear spring damping ratio
		float linear_damping_ratio;

		/// Maximum spring force in newtons
		float max_spring_force;

		/// Angular spring hertz for position control
		float angular_hertz;

		/// Angular spring damping ratio
		float angular_damping_ratio;

		/// Maximum spring torque in newton-meters
		float max_spring_torque;

		MotorJointDesc() noexcept;
	};

	/// A filter joint is used to disable collision between two specific bodies.
	class FilterJointDesc final : public joint_detail::Base
	{
	public:
		FilterJointDesc() noexcept;
	};

	/// Prismatic joint definition
	/// Body B may slide along the x-axis in local frame A. 
	/// Body B cannot rotate relative to body A.
	/// The joint translation is zero when the local frame origins coincide in world space.
	class PrismaticJointDesc final : public joint_detail::Base
	{
	public:
		/// Enable a linear spring along the prismatic joint axis
		bool enable_spring;

		/// The spring stiffness Hertz, cycles per second
		float hertz;

		/// The spring damping ratio, non-dimensional
		float damping_ratio;

		/// The target translation for the joint in meters. The spring-damper will drive
		/// to this translation.
		float target_translation;

		/// Enable/disable the joint limit
		bool enable_limit;

		/// The lower translation limit
		float lower_translation;

		/// The upper translation limit
		float upper_translation;

		/// Enable/disable the joint motor
		bool enable_motor;

		/// The maximum motor force, typically in newtons
		float max_motor_force;

		/// The desired motor speed, typically in meters per second
		float motor_speed;

		PrismaticJointDesc() noexcept;
	};

	/// Revolute joint definition
	/// A point on body B is fixed to a point on body A. Allows relative rotation.
	class RevoluteJointDesc final : public joint_detail::Base
	{
	public:
		/// The target angle for the joint in radians. The spring-damper will drive
		/// to this angle.
		float target_angle;

		/// Enable a rotational spring on the revolute hinge axis
		bool enable_spring;

		/// The spring stiffness Hertz, cycles per second
		float hertz;

		/// The spring damping ratio, non-dimensional
		float damping_ratio;

		/// A flag to enable joint limits
		bool enable_limit;

		/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
		float lower_angle;

		/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
		float upper_angle;

		/// A flag to enable the joint motor
		bool enable_motor;

		/// The maximum motor torque, typically in newton-meters
		float max_motor_torque;

		/// The desired motor speed in radians per second
		float motor_speed;

		RevoluteJointDesc() noexcept;
	};

	/// Weld joint definition
	/// Connects two bodies together rigidly. 
	/// This constraint provides springs to mimic soft-body simulation.
	/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
	class WeldJointDesc final : public joint_detail::Base
	{
	public:
		/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
		float linear_hertz;

		/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
		float angular_hertz;

		/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
		float linear_damping_ratio;

		/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
		float angular_damping_ratio;

		WeldJointDesc() noexcept;
	};

	/// Wheel joint definition
	/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
	/// The joint translation is zero when the local frame origins coincide in world space.
	class WheelJointDesc final : public joint_detail::Base
	{
	public:
		/// Enable a linear spring along the local axis
		bool enable_spring;

		/// Spring stiffness in Hertz
		float hertz;

		/// Spring damping ratio, non-dimensional
		float damping_ratio;

		/// Enable/disable the joint linear limit
		bool enable_limit;

		/// The lower translation limit
		float lower_translation;

		/// The upper translation limit
		float upper_translation;

		/// Enable/disable the joint rotational motor
		bool enable_motor;

		/// The maximum motor torque, typically in newton-meters
		float max_motor_torque;

		/// The desired motor speed in radians per second
		float motor_speed;

		WheelJointDesc() noexcept;
	};
}
