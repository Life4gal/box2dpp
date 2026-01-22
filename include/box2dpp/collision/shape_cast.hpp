// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <optional>

#include <box2dpp/collision/shape_proxy.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	/// Input parameters for casting a generic convex shape along a linear path.
	/// Used for continuous collision detection (CCD) between moving and static shapes.
	class ShapeCastInput final
	{
	public:
		/// Convex shape to cast along the translation vector.
		/// The shape is defined in its local coordinate frame.
		ShapeProxy proxy;

		/// Linear translation vector for the shape cast.
		/// Represents the movement from start to end position.
		Vec2 translation;

		/// Maximum fraction of translation to consider (0 to 1).
		/// Typically, 1.0 for full translation, can be less for early exit.
		float max_fraction;

		/// Allow slight penetration when shapes are initially touching.
		/// Only effective when shapes have positive radius (rounded corners).
		/// This helps prevent tunneling in some edge cases.
		bool can_encroach;

		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// Input parameters for casting shape B relative to shape A.
	/// Shape A is fixed, shape B moves by translation_b.
	/// Both shapes can have arbitrary transforms and radii.
	class ShapeCastPairInput final
	{
	public:
		/// Fixed shape (shape A)
		ShapeProxy proxy_a;

		/// Moving shape (shape B)
		ShapeProxy proxy_b;

		/// World transform for the fixed shape A at time 0
		Transform transform_a;

		/// World transform for the moving shape B at time 0
		Transform transform_b;

		/// Translation vector for shape B from time 0 to time 1
		Vec2 translation_b;

		/// Maximum fraction of translation to consider (0 to 1)
		float max_fraction;

		/// Allow encroachment when shapes are initially touching
		bool can_encroach;

		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// Result of a shape cast operation.
	/// Contains collision information if a hit occurred during the sweep.
	class ShapeCastOutput final
	{
	public:
		/// Surface normal at the hit point (from A to B).
		/// Zero vector in case of initial overlap.
		Vec2 normal;

		/// Contact point in world coordinates at time of collision.
		/// For initial overlap, an arbitrary point on the overlapping region.
		Vec2 point;

		/// Fraction of translation at which collision occurs (0 to max_fraction).
		/// Zero indicates initial overlap, positive indicates time of first contact.
		float fraction;
	};

	/// Continuous collision detection using shape casting (swept test).
	/// Determines if and when a moving shape collides with a static shape along a linear translation path.
	class ShapeCast final
	{
	public:
		/// Cast a generic shape against a stationary circle.
		/// @param input Shape cast parameters for the moving shape
		/// @param circle Stationary circle to test against
		/// @return Shape cast result if collision occurred, null otherwise
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Circle& circle) noexcept -> std::optional<ShapeCastOutput>;

		/// Cast a generic shape against a stationary capsule.
		/// @param input Shape cast parameters for the moving shape
		/// @param capsule Stationary capsule to test against
		/// @return Shape cast result if collision occurred, null otherwise
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Capsule& capsule) noexcept -> std::optional<ShapeCastOutput>;

		/// Cast a generic shape against a stationary convex polygon.
		/// @param input Shape cast parameters for the moving shape
		/// @param polygon Stationary polygon to test against
		/// @return Shape cast result if collision occurred, null otherwise
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Polygon& polygon) noexcept -> std::optional<ShapeCastOutput>;

		/// Cast a generic shape against a stationary line segment.
		/// @param input Shape cast parameters for the moving shape
		/// @param segment Stationary segment to test against
		/// @return Shape cast result if collision occurred, null otherwise
		[[nodiscard]] static auto test(const ShapeCastInput& input, const Segment& segment) noexcept -> std::optional<ShapeCastOutput>;

		/// Perform a shape cast between two arbitrary convex shapes.
		/// Shape A is fixed, shape B moves by translation_b.
		/// @param input Pair-wise shape cast parameters
		/// @return Shape cast result if collision occurred, null otherwise
		[[nodiscard]] static auto test(const ShapeCastPairInput& input) noexcept -> std::optional<ShapeCastOutput>;
	};
}
