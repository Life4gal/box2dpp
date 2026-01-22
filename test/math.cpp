#include <box2dpp/math.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

// 0.0023 degrees
#define ATAN_TOLERANCE 0.00004f

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.vec2"> vec2 = [] noexcept -> void
	{
		using namespace ut;
		using box2dpp::Vec2;

		constexpr Vec2 zero{.x = 0, .y = 0};
		constexpr Vec2 one{.x = 1.f, .y = 1.f};
		constexpr Vec2 two{.x = 2.f, .y = 2.f};
		constexpr Vec2 m_one{.x = -1.f, .y = -1.f};

		"operator-(unary)"_test = [&] noexcept -> void
		{
			constexpr auto o = -m_one;

			expect(o.x == value(-m_one.x));
			expect(o.y == value(-m_one.y));
		};

		"operator-(vec2)"_test = [&] noexcept -> void
		{
			constexpr auto v = zero - two;

			expect(v.x == value(zero.x - two.x));
			expect(v.y == value(zero.y - two.y));
		};

		"operator-(scalar)"_test = [&] noexcept -> void
		{
			constexpr auto v = one - 2;

			expect(v.x == value(one.x - 2));
			expect(v.y == value(one.y - 2));
		};

		"operator+(unary)"_test = [&] noexcept -> void
		{
			const auto o = +m_one;

			expect(o.x == value(std::abs(m_one.x)));
			expect(o.y == value(std::abs(m_one.y)));
		};

		"operator+(vec2)"_test = [&] noexcept -> void
		{
			constexpr auto v = one + two;

			expect(v.x == value(one.x + two.x));
			expect(v.y == value(one.y + two.y));
		};

		"operator+(scalar)"_test = [&] noexcept -> void
		{
			constexpr auto v = one + 2;

			expect(v.x == value(one.x + 2));
			expect(v.y == value(one.y + 2));
		};

		"operator*(vec2)"_test = [&] noexcept -> void
		{
			constexpr auto v = one * two;

			expect(v.x == value(one.x * two.x));
			expect(v.y == value(one.y * two.y));
		};

		"operator*(scalar)"_test = [&] noexcept -> void
		{
			constexpr auto v = one * 2;

			expect(v.x == value(one.x * 2));
			expect(v.y == value(one.y * 2));
		};

		"operator/(vec2)"_test = [&] noexcept -> void
		{
			constexpr auto v = one / two;

			expect(v.x == value(one.x / two.x));
			expect(v.y == value(one.y / two.y));
		};

		"operator/(scalar)"_test = [&] noexcept -> void
		{
			constexpr auto v = one / 2;

			expect(v.x == value(one.x / 2));
			expect(v.y == value(one.y / 2));
		};

		"normalize.zero"_test = [&] noexcept -> void
		{
			constexpr auto v = Vec2{.x = 0.f, .y = 0.f};

			float length;
			const auto n_normalized = v.normalize(length);

			expect(length == value(0.f));
			expect(n_normalized.x == value(0.f));
			expect(n_normalized.y == value(0.f));
		};

		"normalize.tiny_length"_test = [&] noexcept -> void
		{
			constexpr float tiny = std::numeric_limits<float>::denorm_min();
			constexpr auto v = Vec2{.x = tiny, .y = tiny};

			float length = -1.f;
			const auto v_normalized = v.normalize(length);

			expect(length >= value(0.f));
			expect(v_normalized.x == value(0.f));
			expect(v_normalized.y == value(0.f));
		};

		"valid.nan/inf"_test = [&] noexcept -> void
		{
			const Vec2 v_nan{.x = std::nanf(""), .y = 1.f};
			constexpr Vec2 v_inf{.x = 1.f, .y = std::numeric_limits<float>::infinity()};

			expect(v_nan.valid() == value(false));
			expect(v_inf.valid() == value(false));
		};

		"distance.symmetric"_test = [&] noexcept -> void
		{
			constexpr Vec2 a{.x = -3.f, .y = 4.f};
			constexpr Vec2 b{.x = 7.f, .y = 1.f};

			expect(a.distance(b) == value(b.distance(a)));
			expect(a.distance_squared(b) == value(b.distance_squared(a)));
		};

		"lerp.outside_range"_test = [&] noexcept -> void
		{
			constexpr Vec2 a{.x = 1.f, .y = 2.f};
			constexpr Vec2 b{.x = 3.f, .y = 4.f};
			constexpr float epsilon = std::numeric_limits<float>::epsilon() * 10.f;

			const auto r_neg = a.lerp(b, -1.f);
			expect(std::abs(r_neg.x - (2.f * a.x - b.x)) <= value(epsilon));
			expect(std::abs(r_neg.y - (2.f * a.y - b.y)) <= value(epsilon));

			const auto r_big = a.lerp(b, 2.f);
			expect(std::abs(r_big.x - (2.f * b.x - a.x)) <= value(epsilon));
			expect(std::abs(r_big.y - (2.f * b.y - a.y)) <= value(epsilon));
		};

		"combination.min_max"_test = [&] noexcept -> void
		{
			constexpr Vec2 a{.x = -1.f, .y = 5.f};
			constexpr Vec2 b{.x = 2.f, .y = 3.f};

			const auto min = a.combination_min(b);
			const auto max = a.combination_max(b);

			expect(min.x == value(-1.f));
			expect(min.y == value(3.f));
			expect(max.x == value(2.f));
			expect(max.y == value(5.f));
		};

		"multiply_add_sub"_test = [&] noexcept -> void
		{
			constexpr Vec2 a{.x = 1.f, .y = 2.f};
			constexpr Vec2 b{.x = 3.f, .y = 4.f};

			constexpr auto add = box2dpp::multiply_add(a, 2.f, b);
			expect(add.x == value(a.x + 2.f * b.x));
			expect(add.y == value(a.y + 2.f * b.y));

			constexpr auto sub = box2dpp::multiply_sub(a, 2.f, b);
			expect(sub.x == value(a.x - 2.f * b.x));
			expect(sub.y == value(a.y - 2.f * b.y));
		};

		"abs"_test = [&] noexcept -> void
		{
			constexpr Vec2 v{.x = -2.5f, .y = 3.7f};
			const auto a = v.abs();

			expect(a.x == value(2.5f));
			expect(a.y == value(3.7f));
		};

		"reflect"_test = [&] noexcept -> void
		{
			constexpr Vec2 v{.x = 1.f, .y = -1.f};
			constexpr Vec2 normal{.x = 0.f, .y = 1.f};

			const auto reflected = v.reflect(normal);
			// v' = v - 2*(v·n)*n
			// v·n = -1, v' = (1, -1) - 2*(-1)*(0,1) = (1, -1) + (0,2) = (1,1)

			expect(std::abs(reflected.x - 1.f) <= value(1e-5f));
			expect(std::abs(reflected.y - 1.f) <= value(1e-5f));
		};

		"project"_test = [] noexcept -> void
		{
			constexpr Vec2 v{.x = 3.f, .y = 4.f};
			constexpr Vec2 onto{.x = 1.f, .y = 0.f}; // x-axis

			const auto proj = v.project(onto);

			expect(std::abs(proj.x - 3.f) <= value(1e-5f));
			expect(std::abs(proj.y) <= value(1e-5f));

			constexpr Vec2 zero{.x = 0.f, .y = 0.f};
			const auto proj_zero = v.project(zero);

			expect(proj_zero.x == value(0.f));
			expect(proj_zero.y == value(0.f));
		};

		"reject"_test = [] noexcept -> void
		{
			constexpr Vec2 v{.x = 3.f, .y = 4.f};
			constexpr Vec2 onto{.x = 1.f, .y = 0.f}; // x-axis

			const auto proj = v.project(onto);
			const auto rej = v.reject(onto);

			const auto sum = proj + rej;

			expect(std::abs(sum.x - v.x) <= value(1e-5f));
			expect(std::abs(sum.y - v.y) <= value(1e-5f));
			expect(std::abs(proj.dot(rej)) <= value(1e-5f));
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.rotation"> rotation = [] noexcept -> void
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
				expect(std::abs(rotation.cos - cosine) <= .002_f);
				expect(std::abs(rotation.sin - sine) <= .002_f);

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
				expect(std::abs(diff) <= value(ATAN_TOLERANCE));
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
					expect(std::abs(diff) <= value(ATAN_TOLERANCE));
				}
			}
		};

		"0&1"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = 1}.angle();
			const auto angle2 = std::atan2(1.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) <= value(ATAN_TOLERANCE));
		};

		"0&-1"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = -1}.angle();
			const auto angle2 = std::atan2(-1.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) <= value(ATAN_TOLERANCE));
		};

		"1&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 1, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, 1.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) <= value(ATAN_TOLERANCE));
		};

		"-1&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = -1, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, -1.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) <= value(ATAN_TOLERANCE));
		};

		"0&0"_test = [] noexcept -> void
		{
			const auto angle1 = Rotation{.cos = 0, .sin = 0}.angle();
			const auto angle2 = std::atan2(0.f, 0.f);
			const auto diff = angle1 - angle2;

			expect(box2dpp::valid(angle1) == "valid angle"_b);
			expect(std::abs(diff) <= value(ATAN_TOLERANCE));
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

				expect(std::abs(std::numbers::pi_v<float> / 2.f * alpha - angle) <= value(std::numbers::pi_v<float> * 5.f / 180.f));
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
				expect(std::abs(unwound_diff) <= value(tolerance));
			}

			for (auto t = -10.f; t < 10.f; t += .01f)
			{
				const auto angle = std::numbers::pi_v<float> * t;
				const auto q2 = Rotation::from(angle);

				const auto relative_angle = q1.angle(q2);
				const auto unwound_angle = box2dpp::unwind_angle(angle - base_angle);

				expect(std::abs(relative_angle - unwound_angle) <= value(tolerance));
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

					expect(w.x - nu.x <= value(std::numeric_limits<float>::epsilon() * 4.f));
					expect(w.y - nu.y <= value(std::numeric_limits<float>::epsilon() * 4.f));
				}
			}
		};

		"normalize.denormalized"_test = [] noexcept -> void
		{
			// sqrt(0.5) ≈ 0.707
			constexpr Rotation r{.cos = 0.5f, .sin = 0.5f};
			const auto nr = r.normalize();

			expect(nr.normalized() == "normalized"_b);
			expect(std::abs(nr.cos * nr.cos + nr.sin * nr.sin - 1.f) <= value(1e-6f));
		};

		"unwind_angle.large_values"_test = [] noexcept -> void
		{
			constexpr auto two_pi = std::numbers::pi_v<float> * 2.f;

			const auto r1 = box2dpp::unwind_angle(1000.f * two_pi + 1.23f);
			const auto r2 = box2dpp::unwind_angle(-1000.f * two_pi - 2.34f);

			expect(r1 >= value(-std::numbers::pi_v<float>));
			expect(r1 <= value(std::numbers::pi_v<float>));
			expect(r2 >= value(-std::numbers::pi_v<float>));
			expect(r2 <= value(std::numbers::pi_v<float>));
		};

		"from.angle_boundaries"_test = [] noexcept -> void
		{
			constexpr auto tolerance = .002f;

			const auto r_pi = Rotation::from(std::numbers::pi_v<float>);
			const auto r_npi = Rotation::from(-std::numbers::pi_v<float>);
			const auto r_half = Rotation::from(std::numbers::pi_v<float> / 2.f);

			expect(std::abs(r_pi.cos - std::cos(std::numbers::pi_v<float>)) <= value(tolerance));
			expect(std::abs(r_pi.sin - std::sin(std::numbers::pi_v<float>)) <= value(tolerance));
			expect(std::abs(r_npi.cos - std::cos(-std::numbers::pi_v<float>)) <= value(tolerance));
			expect(std::abs(r_npi.sin - std::sin(-std::numbers::pi_v<float>)) <= value(tolerance));
			expect(std::abs(r_half.cos - std::cos(std::numbers::pi_v<float> / 2.f)) <= value(tolerance));
			expect(std::abs(r_half.sin - std::sin(std::numbers::pi_v<float> / 2.f)) <= value(tolerance));
		};

		"from.large_angles"_test = [] noexcept -> void
		{
			constexpr auto huge = 10000.f * std::numbers::pi_v<float>;
			const auto r = Rotation::from(huge);

			expect(r.valid() == "valid rotation"_b);

			const auto angle = r.angle();

			expect(angle >= value(-std::numbers::pi_v<float>));
			expect(angle <= value(std::numbers::pi_v<float>));
		};

		"angle.zero_vector"_test = [] noexcept -> void
		{
			constexpr Rotation r{.cos = 0.f, .sin = 0.f};

			expect(r.angle() == value(0.f));
		};

		"angle.near_boundaries"_test = [] noexcept -> void
		{
			constexpr auto angle_near_pi = std::numbers::pi_v<float> - 1e-5f;
			constexpr auto angle_near_minus_pi = -std::numbers::pi_v<float> + 1e-5f;

			const auto r1 = Rotation::from(angle_near_pi);
			const auto r2 = Rotation::from(angle_near_minus_pi);

			expect(std::abs(r1.angle() - angle_near_pi) <= value(1e-4f));
			expect(std::abs(r2.angle() - angle_near_minus_pi) <= value(1e-4f));
		};

		"angle.precision_comparison"_test = [] noexcept -> void
		{
			constexpr int num_samples = 1000;

			for (int i = 0; i < num_samples; ++i)
			{
				const float angle = -std::numbers::pi_v<float> + (2.f * std::numbers::pi_v<float> * i) / (num_samples - 1);

				const auto r = Rotation::from(angle);
				const auto approx_angle = r.angle();
				const auto std_angle = std::atan2(r.sin, r.cos);

				const auto diff = std::abs(box2dpp::unwind_angle(approx_angle - std_angle));
				expect(diff <= value(ATAN_TOLERANCE * 2.f));
			}
		};

		"axis.orthogonality"_test = [] noexcept -> void
		{
			const auto r = Rotation::from(std::numbers::pi_v<float> / 3.f);
			const auto ax = r.axis_x();
			const auto ay = r.axis_y();

			// dot(ax, ay) == 0
			expect(std::abs(ax.dot(ay)) <= value(std::numeric_limits<float>::epsilon() * 8.f));
		};

		"integrate.zero_delta"_test = [] noexcept -> void
		{
			const auto r = Rotation::from(1.0f);
			const auto r2 = r.integrate(0.0f);

			expect(std::abs(r2.cos - r.cos) <= value(std::numeric_limits<float>::epsilon() * 10.f));
			expect(std::abs(r2.sin - r.sin) <= value(std::numeric_limits<float>::epsilon() * 10.f));
		};

		"integrate.delta"_test = [] noexcept -> void
		{
			const auto r0 = Rotation::from(0.f);
			const auto r1 = r0.integrate(0.1f);

			// approximately a rotation by 0.1 radians
			expect(std::abs(box2dpp::unwind_angle(r1.angle() - 0.1f)) <= value(1e-3f));
		};

		"integrate.large_delta"_test = [] noexcept -> void
		{
			const auto r = Rotation::from(0.0f);
			constexpr auto large_delta = 100.f * std::numbers::pi_v<float>;

			const auto r2 = r.integrate(large_delta);
			expect(r2.valid() == "valid rotation"_b);

			const auto angle = r2.angle();
			expect(angle >= value(-std::numbers::pi_v<float>));
			expect(angle <= value(std::numbers::pi_v<float>));
		};

		"nlerp.opposite_directions"_test = [] noexcept -> void
		{
			const auto r1 = Rotation::from(0.0f);
			const auto r2 = Rotation::from(std::numbers::pi_v<float>);

			const auto mid = r1.nlerp(r2, 0.5f);
			expect(mid.valid() == "valid rotation"_b);

			expect(std::abs(mid.cos) <= value(1e-5f));
			expect(std::abs(std::abs(mid.sin) - 1.f) <= value(1e-5f));
		};

		"nlerp.extremes"_test = [] noexcept -> void
		{
			const auto r1 = Rotation::from(0.0f);
			const auto r2 = Rotation::from(std::numbers::pi_v<float>);

			const auto r_t0 = r1.nlerp(r2, 0.0f);
			expect(std::abs(r_t0.angle() - r1.angle()) <= value(1e-5f));

			const auto r_t1 = r1.nlerp(r2, 1.0f);
			const auto diff = std::abs(r_t1.angle() - r2.angle());
			const auto circular_diff = std::min(diff, std::numbers::pi_v<float> * 2.f - diff);

			expect(circular_diff <= value(1e-5f));
		};

		"nlerp.out_of_bounds_t"_test = [] noexcept -> void
		{
			constexpr auto r0 = Rotation::identity;
			const auto r90 = Rotation::from(std::numbers::pi_v<float> / 2.f);

			// t < 0
			const auto r_neg = r0.nlerp(r90, -0.5f);
			// expect normalized vector pointing opposite direction than simple extrapolation, just verify valid
			expect(r_neg.valid() == "valid angle"_b);

			// t > 1
			const auto r_big = r0.nlerp(r90, 1.5f);
			expect(r_big.valid() == "valid angle"_b);
		};

		"inv_rotate.inverse"_test = [] noexcept -> void
		{
			constexpr Vec2 v{.x = 2.f, .y = -3.f};

			const auto r = Rotation::from(std::numbers::pi_v<float> / 4.f);
			const auto rv = r.rotate(v);
			const auto inv = r.inv_rotate(rv);

			expect(std::abs(inv.x - v.x) <= value(std::numeric_limits<float>::epsilon() * 8.f));
			expect(std::abs(inv.y - v.y) <= value(std::numeric_limits<float>::epsilon() * 8.f));
		};

		"multiply.associativity"_test = [] noexcept -> void
		{
			const auto a = Rotation::from(0.3f);
			const auto b = Rotation::from(-1.1f);
			const auto c = Rotation::from(2.0f);

			const auto left = a.multiply(b).multiply(c);
			const auto right = a.multiply(b.multiply(c));

			expect(std::abs(left.angle() - right.angle()) <= value(1e-5f));
		};

		"multiply.inverse"_test = [] noexcept -> void
		{
			const auto r = Rotation::from(1.5f);
			constexpr auto id = Rotation::identity;

			// r⁻¹ × r = identity
			const auto inv_r = r.inv();
			const auto should_be_id = inv_r.multiply(r);

			expect(std::abs(should_be_id.cos - id.cos) <= value(1e-5f));
			expect(std::abs(should_be_id.sin - id.sin) <= value(1e-5f));

			// r × r⁻¹ = identity
			const auto should_also_be_id = r.multiply(inv_r);
			expect(std::abs(should_also_be_id.cos - id.cos) <= value(1e-5f));
			expect(std::abs(should_also_be_id.sin - id.sin) <= value(1e-5f));
		};

		"multiply_by_inverse"_test = [] noexcept -> void
		{
			const auto a = Rotation::from(0.3f);
			const auto b = Rotation::from(-1.1f);

			// a⁻¹ × b
			const auto m = a.multiply_by_inv(b);

			// a × m = b
			const auto a_times_m = a.multiply(m);
			const auto angle_diff = std::abs(box2dpp::unwind_angle(a_times_m.angle() - b.angle()));
			expect(angle_diff <= value(1e-5f));
		};

		"inv_multiply"_test = [] noexcept -> void
		{
			const auto a = Rotation::from(0.3f);
			const auto b = Rotation::from(-1.1f);

			// b × a⁻¹
			const auto m = a.inv_multiply(b);

			//m × a = b
			const auto m_times_a = m.multiply(a);
			const auto angle_diff = std::abs(box2dpp::unwind_angle(m_times_a.angle() - b.angle()));
			expect(angle_diff <= value(1e-5f));
		};

		get_config().report_level = old_report_level;
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.transform"> transform = [] noexcept -> void
	{
		using namespace ut;
		using box2dpp::Vec2;
		using box2dpp::Rotation;
		using box2dpp::Transform;

		"transform"_test = [] noexcept -> void
		{
			constexpr Vec2 two{.x = 2.f, .y = 2.f};

			const Transform transform1{.point = {.x = -2.f, .y = 3.f}, .rotation = Rotation::from(1.f)};
			const Transform transform2{.point = {.x = 1.f, .y = .0f}, .rotation = Rotation::from(-2.f)};

			"t1.transform(two)"_test = [&] noexcept -> void
			{
				const auto v = transform1.transform(two);
				const auto iv = transform1.inv_transform(v);

				expect(iv.x - two.x <= value(std::numeric_limits<float>::epsilon() * 8.f));
				expect(iv.y - two.y <= value(std::numeric_limits<float>::epsilon() * 8.f));
			};

			"multiply"_test = [&] noexcept -> void
			{
				const auto transform = transform2.multiply(transform1);

				const auto v = transform2.transform(transform1.transform(two));
				const auto u = transform.transform(two);

				expect(u.x - v.x <= value(std::numeric_limits<float>::epsilon() * 10.f));
				expect(u.y - v.y <= value(std::numeric_limits<float>::epsilon() * 10.f));
			};
		};

		"transform_vector"_test = [] noexcept -> void
		{
			const Transform t{.point = {.x = 5.f, .y = -3.f}, .rotation = Rotation::from(0.7f)};
			constexpr Vec2 v{.x = 2.f, .y = 1.f};

			const auto tv = t.transform_vector(v);
			const auto expected = t.rotation.rotate(v);

			expect(std::abs(tv.x - expected.x) <= value(1e-5f));
			expect(std::abs(tv.y - expected.y) <= value(1e-5f));

			const auto itv = t.inv_transform_vector(tv);
			expect(std::abs(itv.x - v.x) <= value(1e-5f));
			expect(std::abs(itv.y - v.y) <= value(1e-5f));
		};

		"identity"_test = [] noexcept -> void
		{
			constexpr auto p = Vec2{.x = 5.f, .y = -7.f};
			constexpr auto id = Transform::identity;
			const auto tp = id.transform(p);

			expect(std::abs(tp.x - p.x) <= value(std::numeric_limits<float>::epsilon() * 8.f));
			expect(std::abs(tp.y - p.y) <= value(std::numeric_limits<float>::epsilon() * 8.f));
		};

		"near_identity"_test = [] noexcept -> void
		{
			constexpr Transform t{.point = {.x = 1e-10f, .y = -1e-10f}, .rotation = Rotation{.cos = 1.0f - 1e-10f, .sin = 1e-10f}};
			constexpr Vec2 v{.x = 1.0f, .y = 2.0f};

			const auto tv = t.transform(v);
			const auto itv = t.inv_transform(tv);

			expect(std::abs(itv.x - v.x) <= value(1e-5f));
			expect(std::abs(itv.y - v.y) <= value(1e-5f));
		};

		"large_translation_inverse_consistency"_test = [] noexcept -> void
		{
			const Transform t{.point = {.x = 1e4f, .y = -1e4f}, .rotation = Rotation::from(0.1f)};

			constexpr std::array<Vec2, 5> test_points
			{{

					{.x = 0.f, .y = 0.f},
					{.x = 1.f, .y = 2.f},
					{.x = -3.f, .y = 5.f},
					{.x = 100.f, .y = -200.f},
					{.x = 0.001f, .y = -0.002f},
			}};

			for (const auto& v: test_points)
			{
				const auto tv = t.transform(v);
				const auto itv = t.inv_transform(tv);

				const auto error = (itv - v).length();
				constexpr float max_allowed_error = 1e-3f;

				expect(error <= value(max_allowed_error));
			}
		};

		"multiply.associativity"_test = [] noexcept -> void
		{
			const Transform transform_a{.point = {.x = 1.f, .y = 0.5f}, .rotation = Rotation::from(0.2f)};
			const Transform transform_b{.point = {.x = -0.7f, .y = 2.3f}, .rotation = Rotation::from(1.1f)};
			const Transform transform_c{.point = {.x = 0.3f, .y = -4.4f}, .rotation = Rotation::from(-0.6f)};

			const auto left = transform_a.multiply(transform_b).multiply(transform_c);
			const auto right = transform_a.multiply(transform_b.multiply(transform_c));

			constexpr auto p = Vec2{.x = 0.9f, .y = -1.2f};
			const auto lp = left.transform(p);
			const auto rp = right.transform(p);

			expect(std::abs(lp.x - rp.x) <= value(std::numeric_limits<float>::epsilon() * 100.f));
			expect(std::abs(lp.y - rp.y) <= value(std::numeric_limits<float>::epsilon() * 100.f));
		};

		"multiply_by_inverse.transform"_test = [] noexcept -> void
		{
			const Transform transform_a{.point = {.x = 1.f, .y = 2.f}, .rotation = Rotation::from(0.5f)};
			const Transform transform_b{.point = {.x = -3.f, .y = 0.7f}, .rotation = Rotation::from(-0.8f)};

			// a⁻¹ × b
			const auto relative = transform_a.multiply_by_inv(transform_b);

			// a × relative = b
			const auto a_times_relative = transform_a.multiply(relative);
			const auto pos_diff = (a_times_relative.point - transform_b.point).length();
			const auto rot_diff = std::abs(box2dpp::unwind_angle(a_times_relative.rotation.angle() - transform_b.rotation.angle()));

			expect(pos_diff <= value(1e-5f));
			expect(rot_diff <= value(1e-5f));
		};

		"inv_multiply.transform"_test = [] noexcept -> void
		{
			const Transform transform_a{.point = {.x = 1.f, .y = 2.f}, .rotation = Rotation::from(0.5f)};
			const Transform transform_b{.point = {.x = -3.f, .y = 0.7f}, .rotation = Rotation::from(-0.8f)};

			// b × a⁻¹
			const auto relative = transform_a.inv_multiply(transform_b);

			// relative × a = b
			const auto relative_multiply_a = relative.multiply(transform_a);
			const auto pos_diff = (relative_multiply_a.point - transform_b.point).length();
			const auto rot_diff = std::abs(box2dpp::unwind_angle(relative_multiply_a.rotation.angle() - transform_b.rotation.angle()));

			expect(pos_diff <= value(1e-5f));
			expect(rot_diff <= value(1e-5f));
		};
	};
}
