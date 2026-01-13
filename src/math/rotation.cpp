// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/math/rotation.hpp>

#include <algorithm>
#include <limits>
#include <numbers>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	auto unwind_angle(const float radians) noexcept -> float
	{
		return std::remainder(radians, std::numbers::pi_v<float> * 2.f);
	}

	auto Rotation::from(const float radians) noexcept -> Rotation
	{
		// return {.cos = std::cos(radians), .sin = std::sin(radians)};

		using std::numbers::pi_v;

		constexpr auto pi2 = pi_v<float> * pi_v<float>;
		constexpr auto hpi = pi_v<float> * .5f;

		const auto x = unwind_angle(radians);

		// cosine needs angle in [-pi/2, pi/2]
		float cosine;
		if (x < -hpi)
		{
			const float y = x + pi_v<float>;
			const float y2 = y * y;

			cosine = -(pi2 - 4.0f * y2) / (pi2 + y2);
		}
		else if (x > hpi)
		{
			const float y = x - pi_v<float>;
			const float y2 = y * y;

			cosine = -(pi2 - 4.0f * y2) / (pi2 + y2);
		}
		else
		{
			const float y2 = x * x;

			cosine = (pi2 - 4.0f * y2) / (pi2 + y2);
		}

		// sine needs angle in [0, pi]
		float sine;
		if (x < 0.0f)
		{
			const float y = x + pi_v<float>;

			sine = -16.0f * y * (pi_v<float> - y) / (5.0f * pi2 - 4.0f * y * (pi_v<float> - y));
		}
		else
		{
			sine = 16.0f * x * (pi_v<float> - x) / (5.0f * pi2 - 4.0f * x * (pi_v<float> - x));
		}

		// Normalize to ensure sin² + cos² ≈ 1
		return Rotation{.cos = cosine, .sin = sine}.normalize();
	}

	auto Rotation::from(const Vec2& unit_vector) noexcept -> Rotation
	{
		BPP_ASSERT(unit_vector.normalized());

		return {.cos = unit_vector.x, .sin = unit_vector.y};
	}

	auto Rotation::from(const Vec2& unit_vector1, const Vec2& unit_vector2) noexcept -> Rotation
	{
		BPP_ASSERT(std::abs(1.f - unit_vector1.length()) < std::numeric_limits<float>::epsilon() * 100.f);
		BPP_ASSERT(std::abs(1.f - unit_vector2.length()) < std::numeric_limits<float>::epsilon() * 100.f);

		const Rotation rotation{.cos = unit_vector1.dot(unit_vector2), .sin = unit_vector1.cross(unit_vector2)};
		return rotation.normalize();
	}

	auto Rotation::valid_angle() const noexcept -> bool
	{
		return box2dpp::valid(cos) and box2dpp::valid(sin);
	}

	auto Rotation::valid() const noexcept -> bool
	{
		return valid_angle() and normalized();
	}

	auto Rotation::normalized() const noexcept -> bool
	{
		const auto value = Vec2{.x = sin, .y = cos}.length_squared();

		return 1.0f - 0.0006f < value and value < 1.0f + 0.0006f;
	}

	auto Rotation::normalize() const noexcept -> Rotation
	{
		BPP_ASSERT(valid_angle());

		const auto length = Vec2{.x = sin, .y = cos}.length();
		if (length <= 0)
		{
			return {.cos = 1, .sin = 0};
		}

		const auto inv_length = 1.f / length;
		return {.cos = cos * inv_length, .sin = sin * inv_length};
	}

	auto Rotation::integrate(const float delta) const noexcept -> Rotation
	{
		BPP_ASSERT(valid_angle());

		const Rotation q2{.cos = cos - sin * delta, .sin = sin + cos * delta};
		return q2.normalize();
	}

	auto Rotation::angle() const noexcept -> float
	{
		BPP_ASSERT(valid_angle());

		// Added check for (0,0) to match atan2 and avoid NaN
		if (cos == 0 and sin == 0) // NOLINT(clang-diagnostic-float-equal)
		{
			return 0;
		}

		// return std::atan2(sin, cos);

		// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4

		const auto y = std::abs(sin);
		const auto x = std::abs(cos);
		const auto mx = std::ranges::max(y, x);
		const auto mn = std::ranges::min(y, x);
		const auto a = mn / mx;

		// Minimax polynomial approximation to atan(a) on [0,1]
		const auto s = a * a;
		const auto c = s * a;
		const auto q = s * s;
		const auto t = -0.094097948f * q - 0.33213072f;

		auto r = 0.024840285f * q + 0.18681418f;
		r = r * s + t;
		r = r * c + a;

		// Map to full circle
		if (y > x)
		{
			r = (std::numbers::pi_v<float> / 2) - r;
		}

		if (cos < 0)
		{
			r = std::numbers::pi_v<float> - r;
		}

		if (sin < 0)
		{
			r = -r;
		}

		return r;
	}

	auto Rotation::angle(const Rotation& other) const noexcept -> float
	{
		const auto c = cos * other.cos + sin * other.sin;
		const auto s = cos * other.sin - sin * other.cos;

		return Rotation{.cos = c, .sin = s}.angle();
	}

	auto Rotation::axis_x() const noexcept -> Vec2
	{
		return {.x = cos, .y = sin};
	}

	auto Rotation::axis_y() const noexcept -> Vec2
	{
		return {.x = -sin, .y = cos};
	}

	// ReSharper disable once IdentifierTypo
	auto Rotation::nlerp(const Rotation& other, const float t) const noexcept -> Rotation
	{
		const auto omt = 1.f - t;
		const auto r = Vec2{.x = cos, .y = sin} * omt + Vec2{.x = other.cos, .y = other.sin} * t;

		const auto mag = r.length();
		const auto inv_mag = mag > 0 ? 1.f / mag : 0;

		return {.cos = r.x * inv_mag, .sin = r.y * inv_mag};
	}

	auto Rotation::inv() const noexcept -> Rotation
	{
		return {.cos = cos, .sin = -sin};
	}

	auto Rotation::rotate(const Vec2& vec2) const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		const auto x = cos * vec2.x - sin * vec2.y;
		const auto y = cos * vec2.y + sin * vec2.x;

		return {.x = x, .y = y};
	}

	auto Rotation::inv_rotate(const Vec2& vec2) const noexcept -> Vec2
	{
		return inv().rotate(vec2);
	}

	auto Rotation::multiply(const Rotation& other) const noexcept -> Rotation
	{
		BPP_ASSERT(valid());

		const auto c = cos * other.cos - sin * other.sin;
		const auto s = sin * other.cos + cos * other.sin;

		return {.cos = c, .sin = s};
	}

	auto Rotation::inv_multiply(const Rotation& other) const noexcept -> Rotation
	{
		return inv().multiply(other);
	}
}
