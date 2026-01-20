// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <box2dpp/math.hpp>

namespace box2dpp
{
	class Circle;
	class Capsule;
	class Polygon;
	class Segment;

	class ShapeCast;

	/// Low level ray cast input data
	class RayCastInput final
	{
	public:
		/// Start point of the ray cast
		Vec2 origin;

		/// Translation of the ray cast
		Vec2 translation;

		/// The maximum fraction of the translation to consider, typically 1
		float max_fraction;

		/// Validate ray cast input data (NaN, etc)
		[[nodiscard]] auto valid() const noexcept -> bool;
	};

	/// Low level ray-cast output data.
	/// Returns a zero fraction and normal in the case of initial overlap.
	class RayCast final // NOLINT(clang-diagnostic-padded)
	{
	public:
		const static RayCast miss;

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

	private:
		[[nodiscard]] static auto from(const ShapeCast& result) noexcept -> RayCast;

	public:
		/// Ray cast versus circle shape in local space.
		[[nodiscard]] static auto test(const RayCastInput& input, const Circle& circle) noexcept -> RayCast;

		/// Ray cast versus capsule shape in local space.
		[[nodiscard]] static auto test(const RayCastInput& input, const Capsule& capsule) noexcept -> RayCast;

		/// Ray cast versus polygon shape in local space.
		[[nodiscard]] static auto test(const RayCastInput& input, const Polygon& polygon) noexcept -> RayCast;

		/// Ray cast versus segment shape in local space.
		/// Optionally treat the segment as one-sided with hits from the left side being treated as a miss.
		[[nodiscard]] static auto test(const RayCastInput& input, const Segment& segment, bool one_sided) noexcept -> RayCast;
	};

	// FIXME: error LNK2005
#ifdef BPP_COMPILER_MSVC
	inline
#endif
	constexpr RayCast RayCast::miss = {.normal = Vec2::zero, .point = Vec2::zero, .fraction = 0.f, .iterations = 0, .hit = false};
}
