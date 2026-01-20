// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <span>

#include <box2dpp/math.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
	/// You can provide between 1 and BPP_MAX_POLYGON_VERTICES and a radius.
	class ShapeProxy final
	{
	public:
		/// The point cloud
		Vec2 points[BPP_MAX_POLYGON_VERTICES];

		/// The number of points.
		std::uint32_t count;

		/// The external radius of the point cloud. 
		/// May be zero.
		float radius;

		/// Make a proxy for use in overlap, shape cast, and related functions.
		/// This is a deep copy of the points.
		[[nodiscard]] static auto create(std::span<const Vec2> points, float radius) noexcept -> ShapeProxy;

		/// Make a proxy with a transform.
		/// This is a deep copy of the points.
		[[nodiscard]] static auto create(std::span<const Vec2> points, float radius, const Transform& transform) noexcept -> ShapeProxy;

		/// Make a proxy with a transform.
		/// This is a deep copy of the points.
		[[nodiscard]] static auto create(std::span<const Vec2> points, float radius, const Vec2& position, const Rotation& rotation) noexcept -> ShapeProxy;

		[[nodiscard]] auto find_support(const Vec2& direction) const noexcept -> std::uint16_t;
	};

	/// Low level shape cast input in generic form.
	/// This allows casting an arbitrary point cloud wrap with a radius.
	/// For example, a circle is a single point with a non-zero radius.
	/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
	class ShapeCastInput final
	{
	public:
		/// A generic shape
		ShapeProxy proxy;

		/// The translation of the shape cast
		Vec2 translation;

		/// The maximum fraction of the translation to consider, typically 1
		float max_fraction;

		/// Allow shape cast to encroach when initially touching.
		/// This only works if the radius is greater than zero.
		bool can_encroach;
	};

	class ShapeCastPairInput final
	{
	public:
		/// The proxy for shape A
		ShapeProxy proxy_a;

		/// The proxy for shape B
		ShapeProxy proxy_b;

		/// The world transform for shape A
		Transform transform_a;

		/// The world transform for shape B
		Transform transform_b;

		/// The translation of shape B
		Vec2 translation_b;

		/// The fraction of the translation to consider, typically 1
		float max_fraction;

		/// Allows shapes with a radius to move slightly closer if already touching
		bool can_encroach;
	};

	/// Low level shape-cast output data.
	/// Returns a zero fraction and normal in the case of initial overlap.
	class ShapeCast final // NOLINT(clang-diagnostic-padded)
	{
	public:
		const static ShapeCast miss;

		/// The surface normal at the hit point
		Vec2 normal;

		/// The surface hit point
		Vec2 point;

		/// The fraction of the input translation at collision
		float fraction;

		/// The number of iterations used
		std::uint32_t iterations;

		/// Did the cast hit?
		bool hit;

		/// Shape cast versus a circle.
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Circle& circle) noexcept -> ShapeCast;

		/// Shape cast versus a capsule.
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Capsule& capsule) noexcept -> ShapeCast;

		/// Shape cast versus a convex polygon.
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Polygon& polygon) noexcept -> ShapeCast;

		/// Shape cast versus a line segment.
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Segment& segment) noexcept -> ShapeCast;

		/// Perform a linear shape cast of shape B moving and shape A fixed.
		/// Determines the hit point, normal, and translation fraction.
		/// Initially touching shapes are treated as a miss.
		[[nodiscard]] static auto test(const ShapeCastPairInput& input) noexcept -> ShapeCast;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr ShapeCast ShapeCast::miss = {.normal = Vec2::zero, .point = Vec2::zero, .fraction = 0.f, .iterations = 0, .hit = false};
}
