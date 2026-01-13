#include <numbers>

#include <box2dpp/math/rotation.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

// 0.0023 degrees
#define ATAN_TOLERANCE 0.00004f

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.rotation"> _ = [] noexcept -> void
	{
		using namespace ut;
		using box2dpp::Vec2;
		using box2dpp::Rotation;

		const auto old_report_level = std::exchange(get_config().report_level, config_type::ReportLevel::ASSERTION_ERROR_ONLY);

		"360"_test = [] noexcept -> void
		{
			constexpr auto min = -10.f;
			constexpr auto max = 10.f;
			constexpr auto step = .01f;
			constexpr auto total_steps = static_cast<std::size_t>((max - min) / step);

			for (std::size_t i = 0; i < total_steps; ++i)
			{
				const auto t = min + step * static_cast<float>(i);
				const auto angle = std::numbers::pi_v<float> * t;

				const auto rotation = Rotation::from(angle);
				const auto cosine = std::cos(angle);
				const auto sine = std::sin(angle);

				// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
				expect(std::abs(rotation.cos - cosine) < .002_f);
				expect(std::abs(rotation.sin - sine) < .002_f);

				const auto xn = box2dpp::unwind_angle(angle);
				expect(xn >= value(-std::numbers::pi_v<float>));
				expect(xn <= value(std::numbers::pi_v<float>));

				const auto a = Rotation{.cos = cosine, .sin = sine}.angle();
				expect(box2dpp::valid(a) == "valid angle"_b);

				auto diff = std::abs(a - xn);
				// The two results can be off by 360 degrees (-pi and pi)
				if (diff > std::numbers::pi_v<float>)
				{
					diff -= std::numbers::pi_v<float> * 2.f;
				}

				// The approximate atan2 is quite accurate
				expect(std::abs(diff) < value(ATAN_TOLERANCE));
			}
		};

		"-1->1"_test = [] noexcept -> void
		{
			constexpr auto min = -1.f;
			constexpr auto max = 1.f;
			constexpr auto step = .01f;
			constexpr auto total_steps = static_cast<std::size_t>((max - min) / step);

			for (std::size_t i_y = 0; i_y < total_steps; ++i_y)
			{
				for (std::size_t i_x = 0; i_x < total_steps; ++i_x)
				{
					const auto x = min + step * static_cast<float>(i_x);
					const auto y = min + step * static_cast<float>(i_y);

					const auto angle1 = Rotation{.cos = x, .sin = y}.angle();
					const auto angle2 = std::atan2(y, x);
					const auto diff = angle1 - angle2;

					expect(box2dpp::valid(angle1) == "valid angle"_b);
					expect(std::abs(diff) < value(ATAN_TOLERANCE));
				}
			}
		};

		"0&1"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = 1}.angle();
			const auto angle2 = std::atan2(1.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) < value(ATAN_TOLERANCE));
		};

		"0&-1"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = -1}.angle();
			const auto angle2 = std::atan2(-1.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) < value(ATAN_TOLERANCE));
		};

		"1&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 1, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, 1.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) < value(ATAN_TOLERANCE));
		};

		"-1&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = -1, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, -1.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) < value(ATAN_TOLERANCE));
		};

		"0&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) < value(ATAN_TOLERANCE));
		};

		// NLerp of b2Rot has an error of over 4 degrees.
		// 2D quaternions should have an error under 1 degree.
		"nlerp"_test = [] noexcept -> void
		{
			constexpr auto q1 = Rotation::identity;
			const auto q2 = Rotation::from(std::numbers::pi_v<float> / 2.f); // 90 degrees

			for (std::size_t i = 0, n = 100; i < n; ++i)
			{
				const auto alpha = static_cast<float>(i) / static_cast<float>(n);
				const auto q = q1.nlerp(q2, alpha);
				const auto angle = q.angle();

				expect(std::abs(std::numbers::pi_v<float> / 2.f * alpha - angle) < value(std::numbers::pi_v<float> * 5.f / 180.f));
			}
		};

		"relative_angle"_test = [] noexcept -> void
		{
			constexpr auto min = -10.f;
			constexpr auto max = 10.f;
			constexpr auto step = .01f;
			constexpr auto total_steps = static_cast<std::size_t>((max - min) / step);
			constexpr auto base_angle = std::numbers::pi_v<float> * .75f;
			// 0.1 degree
			constexpr auto tolerance = std::numbers::pi_v<float> * .1f / 180.f;

			const auto q1 = Rotation::from(base_angle);

			for (std::size_t i = 0; i < total_steps; ++i)
			{
				const auto t = min + step * static_cast<float>(i);
				const auto angle = std::numbers::pi_v<float> * t;
				const auto q2 = Rotation::from(angle);

				const auto relative_angle = q1.angle(q2);
				const auto unwound_angle = box2dpp::unwind_angle(angle - base_angle);

				// expect(std::abs(relative_angle - unwound_angle) < value(tolerance));

				// IMPORTANT: Due to floating-point rounding errors near ±π,
				// two angles that should be equivalent might be computed as π and -π respectively.
				// Since rotations are modulo 2π, π and -π represent the same orientation.
				// Therefore, we need to compare the minimal circular difference rather than direct subtraction.

				// Compute the minimal signed angular difference on the circle
				const auto diff = relative_angle - unwound_angle;
				const auto unwound_diff = box2dpp::unwind_angle(diff);
				expect(std::abs(unwound_diff) < value(tolerance));
			}

			for (auto t = -10.f; t < 10.f; t += .01f)
			{
				const auto angle = std::numbers::pi_v<float> * t;
				const auto q2 = Rotation::from(angle);

				const auto relative_angle = q1.angle(q2);
				const auto unwound_angle = box2dpp::unwind_angle(angle - base_angle);

				expect(std::abs(relative_angle - unwound_angle) < value(tolerance));
			}
		};

		"Vec2::normalize + Rotation::rotate"_test = [] noexcept -> void
		{
			constexpr auto min = -1.f;
			constexpr auto max = 1.f;
			constexpr auto step = .01f;
			constexpr auto total_steps = static_cast<std::size_t>((max - min) / step);

			constexpr Vec2 v{.x = .2f, .y = -.5f};
			const auto nv = v.normalize();

			for (std::size_t i_y = 0; i_y < total_steps; ++i_y)
			{
				for (std::size_t i_x = 0; i_x < total_steps; ++i_x)
				{
					const auto x = min + step * static_cast<float>(i_x);
					const auto y = min + step * static_cast<float>(i_y);

					if (x == 0 and y == 0) // NOLINT(clang-diagnostic-float-equal)
					{
						continue;
					}

					const Vec2 u{.x = x, .y = y};
					const auto nu = u.normalize();

					const auto rotation = Rotation::from(nv, nu);
					const auto w = rotation.rotate(nv);

					expect(w.x - nu.x < value(std::numeric_limits<float>::epsilon() * 4.f));
					expect(w.y - nu.y < value(std::numeric_limits<float>::epsilon() * 4.f));
				}
			}
		};

		get_config().report_level = old_report_level;
	};
}
