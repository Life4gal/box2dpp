// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/desc/filter.hpp>
#include <box2dpp/desc/surface.hpp>

namespace box2dpp
{
	/// Shape type
	enum class ShapeType : std::uint8_t
	{
		/// A circle with an offset
		CIRCLE,

		/// A capsule is an extruded circle
		CAPSULE,

		/// A line segment
		SEGMENT,

		/// A convex polygon
		POLYGON,

		/// A line segment owned by a chain shape
		CHAIN_SEGMENT,
	};

	/// Used to create a shape.
	/// This is a temporary object used to bundle shape creation parameters. 
	/// You may use the same shape definition to create multiple shapes.
	/// @ingroup shape
	class ShapeDesc final
	{
	public:
		BPP_DESC_USER_DATA_DECLARATION

		/// The surface material for this shape.
		SurfaceMaterialDesc material;

		/// The density, usually in kg/m^2.
		/// This is not part of the surface material because this is for the interior, 
		/// which may have other considerations, such as being hollow. 
		/// For example a wood barrel may be hollow or full of water.
		float density;

		/// Collision filtering data.
		FilterDesc filter;

		/// Enable custom filtering. 
		/// Only one of the two shapes needs to enable custom filtering. 
		/// See WorldDesc.
		bool enable_custom_filtering;

		/// A sensor shape generates overlap events but never generates a collision response.
		/// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
		/// Sensors still contribute to the body mass if they have non-zero density.
		/// @note Sensor events are disabled by default.
		/// @see enable_sensor_events
		bool is_sensor;

		/// Enable sensor events for this shape. 
		/// This applies to sensors and non-sensors. 
		/// False by default, even for sensors.
		bool enable_sensor_events;

		/// Enable contact events for this shape. 
		/// Only applies to kinematic and dynamic bodies. 
		/// Ignored for sensors. 
		/// False by default.
		bool enable_contact_events;

		/// Enable hit events for this shape. 
		/// Only applies to kinematic and dynamic bodies. 
		/// Ignored for sensors. 
		/// False by default.
		bool enable_hit_events;

		/// Enable pre-solve contact events for this shape. 
		/// Only applies to dynamic bodies. 
		/// These are expensive
		/// and must be carefully handled due to multithreading. 
		/// Ignored for sensors.
		bool enable_pre_solve_events;

		/// When shapes are created they will scan the environment for collision the next time step. 
		/// This can significantly slow down static body creation when there are many static shapes.
		/// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
		bool invoke_contact_creation;

		/// Should the body update the mass properties when this shape is created. 
		/// Default is true.
		bool update_body_mass;

		ShapeDesc() noexcept;
	};
}
