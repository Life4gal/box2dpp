// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math/vec2.hpp>

#include <box2dpp/desc/filter.hpp>
#include <box2dpp/desc/surface.hpp>

namespace box2dpp
{
	/// Used to create a chain of line segments. 
	/// This is designed to eliminate ghost collisions with some limitations.
	/// - chains are one-sided
	/// - chains have no mass and should be used on static bodies
	/// - chains have a counter-clockwise winding order (normal points right of segment direction)
	/// - chains are either a loop or open
	/// - a chain must have at least 4 points
	/// - the distance between any two points must be greater than B2_LINEAR_SLOP
	/// - a chain shape should not self intersect (this is not validated)
	/// - an open chain shape has NO COLLISION on the first and final edge
	/// - you may overlap two open chains on their first three and/or last three points to get smooth collision
	/// - a chain shape creates multiple line segment shapes on the body
	/// https://en.wikipedia.org/wiki/Polygonal_chain
	/// @warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
	class ChainDesc final
	{
	public:
		BPP_DESC_USER_DATA_DECLARATION

		/// An array of at least 4 points. These are cloned and may be temporary.
		std::span<Vec2> points;

		/// Surface materials for each segment. These are cloned.
		std::span<const SurfaceMaterialDesc> materials;

		/// Contact filtering data.
		/// Determines which other shapes this chain can collide with
		FilterDesc filter;

		/// Indicates a closed chain formed by connecting the first and last points
		/// When true, the chain forms a continuous loop with collision on all edges
		bool is_loop;

		/// Enable sensors to detect this chain. False by default.
		/// When true, sensor events will be generated for overlaps with other sensors
		bool enable_sensor_events;

		ChainDesc() noexcept;
	};
}
