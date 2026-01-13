#include <box2dpp/math/vec2.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.vec2"> _ = [] noexcept -> void
	{
		using namespace ut;
		using box2dpp::Vec2;

		constexpr Vec2 zero{.x = 0, .y = 0};
		constexpr Vec2 one{.x = 1.f, .y = 1.f};
		constexpr Vec2 two{.x = 2.f, .y = 2.f};
		constexpr Vec2 m_one{.x = -1.f, .y = -1.f};

		"operator-(unary)"_test = [] noexcept -> void
		{
			constexpr auto o = -m_one;
			expect(o.x == value(-m_one.x));
			expect(o.y == value(-m_one.y));
		};

		"operator-(vec2)"_test = [] noexcept -> void
		{
			constexpr auto v = zero - two;
			expect(v.x == value(zero.x - two.x));
			expect(v.y == value(zero.y - two.y));
		};

		"operator-(scalar)"_test = [] noexcept -> void
		{
			constexpr auto v = one - 2;
			expect(v.x == value(one.x - 2));
			expect(v.y == value(one.y - 2));
		};

		"operator+(unary)"_test = [] noexcept -> void
		{
			const auto o = +m_one;
			expect(o.x == value(std::abs(m_one.x)));
			expect(o.y == value(std::abs(m_one.y)));
		};

		"operator+(vec2)"_test = [] noexcept -> void
		{
			constexpr auto v = one + two;
			expect(v.x == value(one.x + two.x));
			expect(v.y == value(one.y + two.y));
		};

		"operator+(scalar)"_test = [] noexcept -> void
		{
			constexpr auto v = one + 2;
			expect(v.x == value(one.x + 2));
			expect(v.y == value(one.y + 2));
		};

		"operator*(vec2)"_test = [] noexcept -> void
		{
			constexpr auto v = one * two;
			expect(v.x == value(one.x * two.x));
			expect(v.y == value(one.y * two.y));
		};

		"operator*(scalar)"_test = [] noexcept -> void
		{
			constexpr auto v = one * 2;
			expect(v.x == value(one.x * 2));
			expect(v.y == value(one.y * 2));
		};

		"operator/(vec2)"_test = [] noexcept -> void
		{
			constexpr auto v = one / two;
			expect(v.x == value(one.x / two.x));
			expect(v.y == value(one.y / two.y));
		};

		"operator/(scalar)"_test = [] noexcept -> void
		{
			constexpr auto v = one / 2;
			expect(v.x == value(one.x / 2));
			expect(v.y == value(one.y / 2));
		};
	};
}
