// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <array>
#include <span>
#include <limits>

#include <box2dpp/math/vec2.hpp>
#include <box2dpp/math/rotation.hpp>
#include <box2dpp/math/transform.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	/// A convex shape representation optimized for distance queries using the GJK algorithm.
	/// Wraps point clouds (convex hulls) with an optional radius for rounded shapes.
	/// Can represent circles, capsules, polygons, and other convex shapes.
	/// You can provide between 1 and BPP_MAX_POLYGON_VERTICES and a radius.
	class ShapeProxy final
	{
	public:
		using index_type = std::uint8_t;

		constexpr static auto invalid_index = std::numeric_limits<index_type>::max();

		/// Convex hull vertices in local space. Maximum of BPP_MAX_POLYGON_VERTICES points.
		std::array<Vec2, BPP_MAX_POLYGON_VERTICES> points;

		/// Number of valid vertices in the point cloud (1 to BPP_MAX_POLYGON_VERTICES).
		std::uint32_t count;

		/// Expansion radius for rounded shapes. 
		/// Zero for sharp-edged convex shapes.
		float radius;

		/// Create a proxy from a convex point cloud in local space.
		/// @param points Convex hull vertices (must be convex and non-empty)
		/// @param radius Outer radius for rounded shapes (must be >= 0)
		/// @return Proxy representing the convex shape
		[[nodiscard]] static auto from(std::span<const Vec2> points, float radius) noexcept -> ShapeProxy;

		/// Create proxy from a circle
		[[nodiscard]] static auto from(const Circle& circle) noexcept -> ShapeProxy;

		/// Create proxy from a capsule
		[[nodiscard]] static auto from(const Capsule& capsule) noexcept -> ShapeProxy;

		/// Create proxy from a polygon
		[[nodiscard]] static auto from(const Polygon& polygon) noexcept -> ShapeProxy;

		/// Create proxy from a segment
		[[nodiscard]] static auto from(const Segment& segment) noexcept -> ShapeProxy;

		/// Create a proxy from a convex point cloud transformed to another coordinate frame.
		/// @param points Convex hull vertices in local space
		/// @param radius Outer radius for rounded shapes
		/// @param transform Transformation to apply to all points
		/// @return Proxy with points in the target coordinate frame
		[[nodiscard]] static auto from(std::span<const Vec2> points, float radius, const Transform& transform) noexcept -> ShapeProxy;

		/// Create proxy from a circle
		[[nodiscard]] static auto from(const Circle& circle, const Transform& transform) noexcept -> ShapeProxy;

		/// Create proxy from a capsule
		[[nodiscard]] static auto from(const Capsule& capsule, const Transform& transform) noexcept -> ShapeProxy;

		/// Create proxy from a polygon
		[[nodiscard]] static auto from(const Polygon& polygon, const Transform& transform) noexcept -> ShapeProxy;

		/// Create proxy from a segment
		[[nodiscard]] static auto from(const Segment& segment, const Transform& transform) noexcept -> ShapeProxy;

		/// Check if the proxy is valid for distance computation.
		/// @return true if count > 0, radius >= 0, and all points are valid
		[[nodiscard]] auto valid() const noexcept -> bool;

		/// Find the extreme point (support point) in the given direction.
		/// For a convex shape S, returns point p ∈ S that maximizes p·direction.
		/// @param direction Search direction (does not need to be normalized)
		/// @return Index of the supporting vertex, or 0 if direction is near zero
		[[nodiscard]] auto find_support(const Vec2& direction) const noexcept -> index_type;
	};
}
