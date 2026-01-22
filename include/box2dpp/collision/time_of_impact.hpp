// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/collision/shape_proxy.hpp>

namespace box2dpp
{
	/// Linear and angular motion of a rigid body over a time interval [0, 1].
	/// Used for continuous collision detection between moving bodies.
	/// The sweep interpolates both position and rotation of the body.
	class Sweep final
	{
	public:
		const static Sweep zero;

		/// Local center of mass relative to body origin (constant)
		Vec2 local_center;

		/// World position of center of mass at time 0 (start)
		Vec2 c1;

		/// World position of center of mass at time 1 (end)
		Vec2 c2;

		/// World rotation at time 0 (start)
		Rotation q1;

		/// World rotation at time 1 (end)
		Rotation q2;

		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Evaluate the body transform at a specific time fraction (0 to 1).
		/// @param t Time fraction in [0, 1] (0 = start, 1 = end)
		/// @return Transform at the specified time
		[[nodiscard]] auto transform_of(float t) const noexcept -> Transform;

		/// Get the linear velocity of the center of mass
		[[nodiscard]] auto linear_velocity() const noexcept -> Vec2;

		/// Get the angular displacement (in radians) from start to end
		[[nodiscard]] auto angular_displacement() const noexcept -> float;

		/// Advance the sweep by a time fraction
		/// @param fraction Fraction to advance (0 to 1)
		/// @return Advanced sweep starting from current position
		[[nodiscard]] auto advance(float fraction) const noexcept -> Sweep;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr Sweep Sweep::zero = {.local_center = Vec2::zero, .c1 = Vec2::zero, .c2 = Vec2::zero, .q1 = Rotation::identity, .q2 = Rotation::identity};

	/// Input parameters for time of impact computation between two moving shapes.
	// ReSharper disable once CppInconsistentNaming
	class TOIInput final
	{
	public:
		/// Convex shape A in its local coordinate frame
		ShapeProxy proxy_a;

		/// Convex shape B in its local coordinate frame
		ShapeProxy proxy_b;

		/// Motion of shape A over time interval [0, max_fraction]
		Sweep sweep_a;

		/// Motion of shape B over time interval [0, max_fraction]
		Sweep sweep_b;

		/// Maximum time fraction to consider (typically 1.0)
		/// Defines search interval [0, max_fraction]
		float max_fraction;

		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// Result classification for time of impact computation.
	// ReSharper disable once CppInconsistentNaming
	enum class TOIState : std::uint32_t // NOLINT(performance-enum-size)
	{
		/// Algorithm failed to converge (numerical issues)
		FAILED,

		/// Shapes overlap at time 0 (initial penetration)
		OVERLAPPED,

		/// Collision occurred at computed fraction
		HIT,

		/// No collision in the time interval [0, max_fraction]
		SEPARATED,
	};

	/// Time of impact computation result.
	/// Contains collision details if a hit occurred within the time interval.
	// ReSharper disable once CppInconsistentNaming
	class TOI final
	{
	public:
		/// Classification of the result
		TOIState state;

		/// Contact point in world coordinates at time of collision
		/// Valid when state is HIT or OVERLAPPED
		Vec2 point;

		/// Surface normal at collision (points from shape A to shape B)
		/// Valid when state is HIT
		Vec2 normal;

		/// Time fraction of first contact (0 to max_fraction)
		/// 0 indicates initial overlap, >0 indicates time of first contact
		float fraction;

		/// Estimated separation distance at collision (positive for gap, negative for penetration)
		/// Useful for penetration resolution
		float separation;

		/// Compute the upper bound on time before two shapes penetrate. 
		/// Time is represented as a fraction between [0,tMax]. 
		/// This uses a swept separating axis and may miss some intermediate, non-tunneling collisions. 
		/// If you change the time interval, you should call this function again.
		[[nodiscard]] static auto compute(const TOIInput& input) noexcept -> TOI;
	};
}
