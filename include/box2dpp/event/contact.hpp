// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math/vec2.hpp>

#include <box2dpp/desc/id.hpp>

namespace box2dpp
{
	/// A beginning touch event is generated when two shapes begin touching.
	class ContactBeginTouchEvent final
	{
	public:
		/// The id of the first shape
		ShapeId shape_id_a;

		/// The id of the second shape
		ShapeId shape_id_b;

		/// The transient contact id. 
		/// This contact maybe destroyed automatically when the world is modified or simulated.
		ContactId contact_id;
	};

	/// An end touch event is generated when two shapes stop touching.
	/// You will get an end event if you do anything that destroys contacts previous to the last world step.
	/// These include things like setting the transform, destroying a body or shape, or changing a filter or body type.
	class ContactEndTouchEvent final
	{
	public:
		/// The id of the first shape
		/// @warning this shape may have been destroyed
		ShapeId shape_id_a;

		/// The id of the second shape
		/// @warning this shape may have been destroyed
		ShapeId shape_id_b;

		/// The transient contact id.
		/// @warning this contact may have been destroyed
		ContactId contact_id;
	};

	/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
	/// This may be reported for speculative contacts that have a confirmed impulse.
	class ContactHitEvent final
	{
	public:
		/// The id of the first shape
		ShapeId shape_id_a;

		/// The id of the second shape
		ShapeId shape_id_b;

		/// Point where the shapes hit at the beginning of the time step.
		/// This is a mid-point between the two surfaces. 
		/// It could be at speculative point where the two shapes were not touching at the beginning of the time step.
		Vec2 point;

		/// Normal vector pointing from shape A to shape B
		Vec2 normal;

		/// The speed the shapes are approaching. 
		/// Always positive. 
		/// Typically, in meters per second.
		float approach_speed;
	};

	/// Contact events are buffered in the world and are available
	/// as event arrays after the time step is complete.
	/// Note: these may become invalid if bodies and/or shapes are destroyed
	class ContactEvents
	{
	public:
		std::span<const ContactBeginTouchEvent> begins;
		std::span<const ContactEndTouchEvent> ends;
		std::span<const ContactHitEvent> hits;
	};
}
