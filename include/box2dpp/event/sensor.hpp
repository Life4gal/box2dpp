// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// A beginning touch event is generated when a shape starts to overlap a sensor shape.
	class SensorBeginTouchEvent final
	{
	public:
		/// The id of the sensor shape
		ShapeId sensor_shape_id;

		/// The id of the shape that began touching the sensor shape
		ShapeId visitor_shape_id;
	};

	/// An end touch event is generated when a shape stops overlapping a sensor shape.
	/// These include things like setting the transform, destroying a body or shape, or changing a filter. 
	/// You will also get an end event if the sensor or visitor are destroyed.
	/// Therefore, you should always confirm the shape id is valid. 
	class SensorEndTouchEvent final
	{
	public:
		/// The id of the sensor shape
		/// @warning this shape may have been destroyed
		ShapeId sensor_shape_id;

		/// The id of the shape that stopped touching the sensor shape
		/// @warning this shape may have been destroyed
		ShapeId visitor_shape_id;
	};

	/// Sensor touch events are buffered in the world and are available
	/// as begin/end overlap event arrays after the time step is complete.
	/// Note: these may become invalid if bodies and/or shapes are destroyed
	class SensorTouchEvents final
	{
	public:
		std::span<const SensorBeginTouchEvent> begins;
		std::span<const SensorEndTouchEvent> ends;
	};
}
