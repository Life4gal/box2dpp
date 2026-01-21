// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <array>
#include <span>

#include <box2dpp/math.hpp>

namespace box2dpp
{
	/// A convex hull. Used to create convex polygons.
	class Hull final
	{
	public:
		/// The final points of the hull
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> points;

		/// The number of points
		std::uint32_t count;

	private:
		[[nodiscard]] static auto recurse_create(const Vec2& p1, const Vec2& p2, std::span<const Vec2> points) noexcept -> Hull;

	public:
		/// Compute the convex hull of a set of points. 
		/// Returns an empty hull if it fails.
		/// Some failure cases:
		/// - all points very close together
		/// - all points on a line
		/// - less than 3 points
		/// - more than BPP_MAX_POLYGON_VERTICES points
		/// This welds close points and removes collinear points.
		/// @warning Do not modify a hull once it has been computed
		[[nodiscard]] static auto create(std::span<const Vec2> points) noexcept -> Hull;

		/// This determines if a hull is valid. 
		/// Checks for:
		/// - convexity
		/// - collinear points
		/// This is expensive and should not be called at runtime.
		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// A solid convex polygon. 
	/// It is assumed that the interior of the polygon is to the left of each edge.
	/// Polygons have a maximum number of vertices equal to BPP_MAX_POLYGON_VERTICES.
	/// In most cases you should not need many vertices for a convex polygon.
	/// @warning DO NOT fill this out manually, instead use a helper function.
	class Polygon final
	{
	public:
		/// The polygon vertices
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> vertices;

		/// The outward normal vectors of the polygon sides
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> normals;

		/// The centroid of the polygon
		Vec2 centroid;

		/// The external radius for rounded polygons
		float radius;

		/// The number of polygon vertices
		std::uint32_t count;

	private:
		[[nodiscard]] static auto compute_centroid(std::span<const Vec2> vertices) noexcept -> Vec2;

	public:
		/// Make a convex polygon from a convex hull. 
		/// This will assert if the hull is not valid.
		[[nodiscard]] static auto make(const Hull& hull, float radius) noexcept -> Polygon;

		/// Make an offset convex polygon from a convex hull. 
		/// This will assert if the hull is not valid.
		[[nodiscard]] static auto make(const Hull& hull, const Vec2& position, const Rotation& rotation) noexcept -> Polygon;

		/// Make an offset convex polygon from a convex hull. 
		/// This will assert if the hull is not valid.
		[[nodiscard]] static auto make(const Hull& hull, const Vec2& position, const Rotation& rotation, float radius) noexcept -> Polygon;

		/// Make a square polygon, bypassing the need for a convex hull.
		/// @param half_width the half-width
		[[nodiscard]] static auto make_square(float half_width) noexcept -> Polygon;

		/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
		/// @param half_width the half-width (x-axis)
		/// @param half_height the half-height (y-axis)
		[[nodiscard]] static auto make_box(float half_width, float half_height) noexcept -> Polygon;

		/// Make a rounded box, bypassing the need for a convex hull.
		/// @param half_width the half-width (x-axis)
		/// @param half_height the half-height (y-axis)
		/// @param radius the radius of the rounded extension
		[[nodiscard]] static auto make_box(float half_width, float half_height, float radius) noexcept -> Polygon;

		/// Make an offset box, bypassing the need for a convex hull.
		/// @param half_width the half-width (x-axis)
		/// @param half_height the half-height (y-axis)
		/// @param center the local center of the box
		/// @param rotation the local rotation of the box
		[[nodiscard]] static auto make_box(float half_width, float half_height, const Vec2& center, const Rotation& rotation) noexcept -> Polygon;

		/// Make an offset rounded box, bypassing the need for a convex hull.
		/// @param half_width the half-width (x-axis)
		/// @param half_height the half-height (y-axis)
		/// @param center the local center of the box
		/// @param rotation the local rotation of the box
		/// @param radius the radius of the rounded extension
		[[nodiscard]] static auto make_box(float half_width, float half_height, const Vec2& center, const Rotation& rotation, float radius) noexcept -> Polygon;

		/// Transform a polygon. 
		/// This is useful for transferring a shape from one body to another.
		[[nodiscard]] static auto transform(const Transform& transform, const Polygon& polygon) noexcept -> Polygon;

		/// Test a point for overlap with a convex polygon in local space
		[[nodiscard]] auto in(const Vec2& point) const noexcept -> bool;
	};
}
