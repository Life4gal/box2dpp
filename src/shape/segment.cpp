// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/shape/segment.hpp>

#include <algorithm>
#include <limits>
#include <ranges>

namespace box2dpp
{
	auto SegmentDistance::compute(const Segment& segment1, const Segment& segment2) noexcept -> SegmentDistance
	{
		return compute(segment1.point1, segment1.point2, segment2.point1, segment2.point2);
	}

	auto SegmentDistance::compute(const Vec2& p1, const Vec2& q1, const Vec2& p2, const Vec2& q2) noexcept -> SegmentDistance
	{
		const auto d1 = q1 - p1;
		const auto d2 = q2 - p2;
		const auto r = p1 - p2;
		const auto dd1 = d1.dot(d1);
		const auto dd2 = d2.dot(d2);
		const auto rd1 = r.dot(d1);
		const auto rd2 = r.dot(d2);

		constexpr auto epsilon2 = std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon();

		float fraction1;
		float fraction2;

		if (dd1 < epsilon2 or dd2 < epsilon2)
		{
			// Handle all degeneracies
			if (dd1 >= epsilon2)
			{
				// Segment 2 is degenerate
				fraction1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				fraction2 = 0;
			}
			else if (dd2 >= epsilon2)
			{
				fraction1 = 0;
				fraction2 = std::ranges::clamp(rd2 / dd2, 0.f, 1.f);
			}
			else
			{
				fraction1 = 0;
				fraction2 = 0;
			}
		}
		else
		{
			// Non-degenerate segments
			const auto d12 = d1.dot(d2);
			const auto denominator = dd1 * dd2 - d12 * d12;

			// Fraction on segment 1
			float f1 = 0;
			if (denominator != 0) // NOLINT(clang-diagnostic-float-equal)
			{
				// not parallel
				f1 = std::ranges::clamp((d12 * rd2 - dd2 * rd1) / denominator, 0.f, 1.f);
			}

			// Compute point on segment 2 closest to p1 + f1 * d1
			auto f2 = (d12 * f1 + rd2) / dd2;

			// Clamping of segment 2 requires a do-over on segment 1
			if (f2 < 0)
			{
				f1 = std::ranges::clamp(-rd1 / dd1, 0.f, 1.f);
				f2 = 0;
			}
			else if (f2 > 1)
			{
				f1 = std::ranges::clamp((d12 - rd1) / dd1, 0.f, 1.f);
				f2 = 1;
			}

			fraction1 = f1;
			fraction2 = f2;
		}

		const auto closest1 = multiply_add(p1, fraction1, d1);
		const auto closest2 = multiply_add(p2, fraction2, d2);
		const auto distance_squared = closest1.distance_squared(closest2);

		return {.closest1 = closest1, .closest2 = closest2, .fraction1 = fraction1, .fraction2 = fraction2, .distance_squared = distance_squared};
	}
}
