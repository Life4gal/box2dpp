// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math/vec2.hpp>
#include <box2dpp/math/rotation.hpp>

namespace box2dpp
{
	/// The body simulation type.
	/// Each body is one of these three types. 
	/// The type determines how the body behaves in the simulation.
	enum class BodyType : std::uint8_t
	{
		/// zero mass, zero velocity, may be manually moved
		STATIC,
		/// zero mass, velocity set by user, moved by solver
		KINEMATIC,

		DYNAMIC,
	};

	/// Motion locks to restrict the body movement
	enum class MotionLocks : std::uint8_t
	{
		NONE = 0b0000'0000,
		/// Prevent translation along the x-axis
		X = 0b0000'0001,
		/// Prevent translation along the y-axis
		Y = 0b0000'0010,
		/// Prevent rotation around the z-axis
		Z = 0b0000'0100,

		XY = X | Y,
		XZ = X | Z,
		YZ = Y | Z,
		XYZ = X | Y | Z,

		ALL = XYZ,
	};

	/// A body definition holds all the data needed to construct a rigid body.
	/// You can safely re-use body definitions. Shapes are added to a body after construction.
	/// Body definitions are temporary objects used to bundle creation parameters.
	class BodyDesc final
	{
	public:
		/// The body type: static, kinematic, or dynamic.
		BodyType type;

		/// The initial world position of the body. Bodies should be created with the desired position.
		/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, 
		/// especially if the body is moved after shapes have been added.
		Vec2 position;

		/// The initial world rotation of the body. Use make_rotation() if you have an angle.
		Rotation rotation;

		/// The initial linear velocity of the body's origin. Usually in meters per second.
		Vec2 linear_velocity;

		/// The initial angular velocity of the body. Radians per second.
		float angular_velocity;

		/// Linear damping is used to reduce the linear velocity. 
		/// The damping parameter can be larger than 1 but the damping effect 
		/// becomes sensitive to the time step when the damping parameter is large.
		/// Generally linear damping is undesirable because it makes objects move slowly as if they are floating.
		float linear_damping;

		/// Angular damping is used to reduce the angular velocity. 
		/// The damping parameter can be larger than 1.0f but the damping effect 
		/// becomes sensitive to the time step when the damping parameter is large.
		/// Angular damping can be use slow down rotating bodies.
		float angular_damping;

		/// Scale the gravity applied to this body. Non-dimensional.
		float gravity_scale;

		/// Sleep speed threshold, default is 0.05 meters per second
		/// When the linear velocity magnitude falls below this threshold, the body may go to sleep
		/// Sleeping bodies are excluded from simulation to improve performance
		float sleep_threshold;

		/// Motions locks to restrict linear and angular movement.
		/// Caution: may lead to softer constraints along the locked direction
		MotionLocks motion_locks;

		/// Set this flag to false if this body should never fall asleep.
		/// Useful for objects that need continuous simulation regardless of velocity
		bool enable_sleep;

		/// Is this body initially awake or sleeping?
		/// If false, the body starts in sleeping state and won't be simulated until woken up
		bool is_awake;

		/// Treat this body as high speed object that performs continuous collision detection
		/// against dynamic and kinematic bodies, but not other bullet bodies.
		/// @warning Bullets should be used sparingly. 
		/// They are not a solution for general dynamic-versus-dynamic continuous collision.
		/// @note Enabling this flag increases computational cost but prevents tunneling for fast-moving objects
		bool is_bullet;

		/// Used to disable a body. A disabled body does not move or collide.
		/// Can be used to temporarily remove a body from simulation without destroying it
		bool is_enabled;

		/// This allows this body to bypass rotational speed limits. 
		/// Should only be used for circular objects, like wheels.
		/// @note Normally, angular velocity is clamped to prevent numerical instability
		bool allow_fast_rotation;

		BodyDesc() noexcept;
	};
}
