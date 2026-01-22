#include <box2dpp/collision.hpp>
#include <box2dpp/shape.hpp>

#include <numbers>
#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.ray_cast"> ray_cast = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Vec2;
		using box2dpp::RayCastInput;
		using box2dpp::RayCast;

		"circle"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			"Horizontal ray through circle center"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = -4, .y = 0}, .translation = {.x = 8, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(-1,0), point=(0,0), fraction=0.5
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Vertical ray through circle center"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 2}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = 0, .y = -2}, .translation = {.x = 0, .y = 8}, .max_fraction = 1};

				// hit=true, normal=(0,-1), point=(0,1), fraction=0.375
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .375f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray starts inside circle (initial overlap)"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 2};
				constexpr RayCastInput input{.origin = {.x = .5f, .y = .5f}, .translation = {.x = 1, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(0,0), point=(0.5,0.5), fraction=0
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + -.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray tangent to circle"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 2}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = -3, .y = 1}, .translation = {.x = 6, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(0,-1), point=(0,1), fraction=0.5
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray completely misses circle"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = 3, .y = 3}, .translation = {.x = 1, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() != "miss"_b);
			};

			"Ray points away from circle"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = 2, .y = 0}, .translation = {.x = 1, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, circle);

				expect(output.has_value() != "miss"_b);
			};

			"Zero-length ray outside circle"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = 2, .y = 0}, .translation = {.x = 0, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, circle);

				expect(output.has_value() != "miss"_b);
			};

			"Zero-length ray inside circle"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = .5f, .y = 0}, .translation = {.x = 0, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(0,0), point=(0.5,0), fraction=0
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray ends exactly on circle edge"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = 2, .y = 0}, .translation = {.x = -1, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(1,0), point=(1,0), fraction=1
				const auto output = RayCast::test(input, circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"capsule"_test = [] noexcept -> void
		{
			using box2dpp::Capsule;

			"Horizontal ray through capsule center"_test = [] noexcept -> void
			{
				constexpr Capsule capsule{.center1 = {.x = -1, .y = 0}, .center2 = {.x = 1, .y = 0}, .radius = .5f};
				constexpr RayCastInput input{.origin = {.x = -4, .y = 0}, .translation = {.x = 8, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(-1,0), point=(-1.5,0), fraction=0.3125
				const auto output = RayCast::test(input, capsule);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 1.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .3125f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Vertical ray through capsule rectangle"_test = [] noexcept -> void
			{
				constexpr Capsule capsule{.center1 = {.x = 0, .y = -1}, .center2 = {.x = 0, .y = 1}, .radius = .5f};
				constexpr RayCastInput input{.origin = {.x = 0, .y = -3}, .translation = {.x = 0, .y = 6}, .max_fraction = 1};

				// hit=true, normal=(0,-1), point=(0,-1.5), fraction=0.25
				const auto output = RayCast::test(input, capsule);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 1.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .25f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray tangent to capsule side"_test = [] noexcept -> void
			{
				constexpr Capsule capsule{.center1 = {.x = -2, .y = 0}, .center2 = {.x = 2, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = -3, .y = 1}, .translation = {.x = 6, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, capsule);
				expect(output.has_value() != "miss"_b);
			};

			"Ray hits capsule end cap"_test = [] noexcept -> void
			{
				constexpr Capsule capsule{.center1 = {.x = -2, .y = 0}, .center2 = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = -4, .y = 0}, .translation = {.x = 8, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(-1,0), point=(-3,0), fraction=0.125
				const auto output = RayCast::test(input, capsule);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 3.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .125f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Degenerate capsule (circle)"_test = [] noexcept -> void
			{
				constexpr Capsule capsule{.center1 = {.x = 0, .y = 0}, .center2 = {.x = 0, .y = 0}, .radius = 1};
				constexpr RayCastInput input{.origin = {.x = -2, .y = 0}, .translation = {.x = 4, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(-1,0), point=(-1,0), fraction=0.25
				const auto output = RayCast::test(input, capsule);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .25f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"polygon(box)"_test = [] noexcept -> void
		{
			using box2dpp::Polygon;

			const auto box = Polygon::make_square(1);

			"Ray hits square from left side"_test = [&] noexcept -> void
			{
				constexpr RayCastInput input{.origin = {.x = -3, .y = 0}, .translation = {.x = 6, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(-1,0), point=(-1,0), fraction=0.333...
				const auto output = RayCast::test(input, box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 1.f / 3.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray hits square from top side"_test = [&] noexcept -> void
			{
				constexpr RayCastInput input{.origin = {.x = 0, .y = 3}, .translation = {.x = 0, .y = -6}, .max_fraction = 1};

				// hit=true, normal=(0,1), point=(0,1), fraction=0.333...
				const auto output = RayCast::test(input, box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 1.f / 3.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray hits near square corner"_test = [&] noexcept -> void
			{
				constexpr RayCastInput input{.origin = {.x = 2, .y = 2}, .translation = {.x = -4, .y = -4}, .max_fraction = 1};

				// hit=true, normal=(1,0), point=(1,1), fraction=0.25
				const auto output = RayCast::test(input, box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .25f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray starts inside polygon"_test = [&] noexcept -> void
			{
				constexpr RayCastInput input{.origin = {.x = 0, .y = 0}, .translation = {.x = 1, .y = 0}, .max_fraction = 1};

				// hit=true, normal=(0,0), point=(0,0), fraction=0
				const auto output = RayCast::test(input, box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray parallel to edge outside polygon"_test = [&] noexcept -> void
			{
				constexpr RayCastInput input{.origin = {.x = -2, .y = 2}, .translation = {.x = 4, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, box);
				expect(output.has_value() != "miss"_b);
			};
		};

		"segment"_test = [] noexcept -> void
		{
			using box2dpp::Segment;

			"Ray hits two-sided segment"_test = [] noexcept -> void
			{
				constexpr Segment segment{.point1 = {.x = -1, .y = -1}, .point2 = {.x = 1, .y = 1}};
				constexpr RayCastInput input{.origin = {.x = -2, .y = 0}, .translation = {.x = 4, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output1 = RayCast::test(input, segment, true);
				// hit=true, normal=(-0.707106769,0.707106769), point=(0,0), fraction=0.5
				const auto output2 = RayCast::test(input, segment, false);

				expect(output1.has_value() != "miss"_b);
				expect(output2.has_value() == "hit"_b);

				const auto& r = *output2;
				expect(std::abs(r.normal.x + std::numbers::sqrt2_v<float> / 2) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + -std::numbers::sqrt2_v<float> / 2) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray hits one-sided segment from right side"_test = [] noexcept -> void
			{
				constexpr Segment segment{.point1 = {.x = -1, .y = 0}, .point2 = {.x = 1, .y = 0}};
				constexpr RayCastInput input{.origin = {.x = 0, .y = 2}, .translation = {.x = 0, .y = -4}, .max_fraction = 1};

				// hit=false
				const auto output1 = RayCast::test(input, segment, true);
				// hit=true, normal=(0,1), point=(0,0), fraction=0.5
				const auto output2 = RayCast::test(input, segment, false);

				expect(output1.has_value() != "miss"_b);
				expect(output2.has_value() == "hit"_b);

				const auto& r = *output2;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray hits one-sided segment from left side"_test = [] noexcept -> void
			{
				constexpr Segment segment{.point1 = {.x = -1, .y = 0}, .point2 = {.x = 1, .y = 0}};
				constexpr RayCastInput input{.origin = {.x = 0, .y = -2}, .translation = {.x = 0, .y = 4}, .max_fraction = 1};

				// hit=true, normal=(0,-1), point=(0,0), fraction=0.5
				const auto output1 = RayCast::test(input, segment, true);
				// hit=true, normal=(0,-1), point=(0,0), fraction=0.5
				const auto output2 = RayCast::test(input, segment, false);
				expect(output1.has_value() == "hit"_b);
				expect(output2.has_value() == "hit"_b);

				const auto& r1 = *output1;

				expect(std::abs(r1.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r1.normal.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r1.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r1.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r1.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));

				const auto& r2 = *output2;
				expect(std::abs(r2.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r2.normal.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r2.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r2.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r2.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Ray parallel to segment"_test = [] noexcept -> void
			{
				constexpr Segment segment{.point1 = {.x = 0, .y = 1}, .point2 = {.x = 4, .y = 1}};
				constexpr RayCastInput input{.origin = {.x = 0, .y = 2}, .translation = {.x = 4, .y = 0}, .max_fraction = 1};

				// hit=false
				const auto output = RayCast::test(input, segment, false);
				expect(output.has_value() != "miss"_b);
			};

			"Ray hits segment endpoint"_test = [] noexcept -> void
			{
				constexpr Segment segment{.point1 = {.x = 0, .y = 0}, .point2 = {.x = 2, .y = 0}};
				constexpr RayCastInput input{.origin = {.x = 1, .y = 2}, .translation = {.x = 0, .y = -4}, .max_fraction = 1};

				// hit=false
				const auto output1 = RayCast::test(input, segment, true);
				// hit=true, normal=(0,1), point=(1,0), fraction=0.5
				const auto output2 = RayCast::test(input, segment, false);

				expect(output1.has_value() != "miss"_b);
				expect(output2.has_value() == "hit"_b);

				const auto& r = *output2;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .5f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"boundary"_test = [] noexcept -> void
		{
			"circle"_test = [] noexcept -> void
			{
				using box2dpp::Circle;

				"max_fraction limits intersection"_test = [] noexcept -> void
				{
					constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
					constexpr RayCastInput input{.origin = {.x = -3, .y = 0}, .translation = {.x = 6, .y = 0}, .max_fraction = .3f};

					// hit=false (fraction=0.333... > max_fraction)
					const auto output = RayCast::test(input, circle);
					expect(output.has_value() != "miss"_b);
				};

				"Large coordinates"_test = [] noexcept -> void
				{
					constexpr Circle circle{.center = {.x = 1000000, .y = 1000000}, .radius = 100};
					constexpr RayCastInput input{.origin = {.x = 999000, .y = 1000000}, .translation = {.x = 2000, .y = 0}, .max_fraction = 1};

					// hit=true, normal=(-1,0), point=(999900,1000000), fraction=0.45
					const auto output = RayCast::test(input, circle);
					expect(output.has_value() == "hit"_b);

					const auto& r = *output;
					expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.point.x + -999900.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.point.y + -1000000.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.fraction - .45f) <= value(std::numeric_limits<float>::epsilon()));
				};

				"Tiny coordinates"_test = [] noexcept -> void
				{
					constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = .001f};
					constexpr RayCastInput input{.origin = {.x = -.002f, .y = 0}, .translation = {.x = .004f, .y = 0}, .max_fraction = 1};

					// hit=true, normal=(-1,0), point=(-0.001,0), fraction=0.25
					const auto output = RayCast::test(input, circle);
					expect(output.has_value() == "hit"_b);

					const auto& r = *output;
					expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.point.x + .001f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
					expect(std::abs(r.fraction - .25f) <= value(std::numeric_limits<float>::epsilon()));
				};
			};
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.shape_cast"> shape_cast = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Vec2;
		using box2dpp::Rotation;
		using box2dpp::Transform;
		using box2dpp::ShapeProxy;
		using box2dpp::ShapeCastInput;
		using box2dpp::ShapeCastPairInput;
		using box2dpp::ShapeCastOutput;
		using box2dpp::ShapeCast;

		"circle"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			"Moving circle hits static circle"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 5, .y = 0}, .radius = 1};
				constexpr Vec2 translation{.x = 10, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(-1,0), point=(4,0), fraction=0.3005
				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -4.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .3005f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Moving circle misses static circle"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 5, .y = 5}, .radius = 1};
				constexpr Vec2 translation{.x = 10, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() != "miss"_b);
			};

			"Circle initially overlapping"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 2};
				constexpr Circle static_circle{.center = {.x = 1, .y = 0}, .radius = 2};
				constexpr Vec2 translation{.x = 5, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(0,0), point=(0.5,0), fraction=0
				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Tangent hit (grazing contact)"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 2}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 5, .y = 0}, .radius = 1};
				constexpr Vec2 translation{.x = 10, .y = -2};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() == "hit"_b);
			};

			"Zero translation"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 3, .y = 0}, .radius = 1};
				constexpr Vec2 translation{.x = 0, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() != "miss"_b);
			};
		};

		"box"_test = [] noexcept -> void
		{
			using box2dpp::Polygon;

			"Moving box hits static box"_test = [] noexcept -> void
			{
				const auto moving_box = Polygon::make_box(1.0f, 0.5f);
				const auto static_box = Polygon::make_box(1.0f, 0.5f);
				constexpr Vec2 translation{.x = 5, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_box),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(0,0), point=(-1,-0.5), fraction=0
				auto output = ShapeCast::test(input, static_box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + .5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Rotated box cast"_test = [] noexcept -> void
			{
				constexpr auto degree_45 = 45.f * std::numbers::pi_v<float> / 180.f;
				constexpr auto degree_22_5 = 22.5f * std::numbers::pi_v<float> / 180.f;
				const auto tan_22_5 = std::tan(degree_22_5);

				const auto moving_box = Polygon::make_box(1.0f, 1.0f);
				const auto static_box = Polygon::make_box(1.0f, 1.0f, {.x = 0, .y = 0}, Rotation::from(degree_45)); // 45 degrees
				constexpr Vec2 translation{.x = 3, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_box),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(0,0), point=(-0.414213687,-1), fraction=0
				auto output = ShapeCast::test(input, static_box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + tan_22_5) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"capsule"_test = [] noexcept -> void
		{
			using box2dpp::Capsule;

			"Capsule vs capsule"_test = [] noexcept -> void
			{
				constexpr Capsule moving_capsule{.center1 = {.x = -1, .y = 0}, .center2 = {.x = 1, .y = 0}, .radius = 0.5f};
				constexpr Capsule static_capsule{.center1 = {.x = 3, .y = -1}, .center2 = {.x = 3, .y = 1}, .radius = 0.5f};
				constexpr Vec2 translation{.x = 5, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_capsule),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(-1,0), point=(2.5,0), fraction=0.201
				const auto output = ShapeCast::test(input, static_capsule);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + -2.5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - .201f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Zero-length capsule (circle)"_test = [] noexcept -> void
			{
				using box2dpp::Polygon;

				constexpr Capsule capsule{.center1 = {.x = 0, .y = 0}, .center2 = {.x = 0, .y = 0}, .radius = 1}; // Degenerate capsule = circle
				const Polygon static_box = Polygon::make_box(1.0f, 1.0f);
				constexpr Vec2 translation{.x = 3, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(capsule),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				// hit=true, normal=(0,0), point=(0,0), fraction=0
				const auto output = ShapeCast::test(input, static_box);
				expect(output.has_value() == "hit"_b);

				const auto& r = *output;
				expect(std::abs(r.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.point.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(r.fraction - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"edge_cases"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			"Very small translation"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 2.1f, .y = 0}, .radius = 1}; // 0.1 gap
				constexpr Vec2 translation{.x = 0.05f, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() != "miss"_b);
			};

			"Max fraction less than hit time"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle static_circle{.center = {.x = 5, .y = 0}, .radius = 1};
				constexpr Vec2 translation{.x = 10, .y = 0};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 0.2f, // Only check first 20% of movement
						.can_encroach = false
				};

				// fraction=0.3 > max_fraction
				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() != "miss"_b);
			};

			"Large shapes, large translation"_test = [] noexcept -> void
			{
				constexpr Circle moving_circle{.center = {.x = 0, .y = 0}, .radius = 100};
				constexpr Circle static_circle{.center = {.x = 500, .y = 500}, .radius = 100};
				constexpr Vec2 translation{.x = 1000, .y = 1000};

				const ShapeCastInput input
				{
						.proxy = ShapeProxy::from(moving_circle),
						.translation = translation,
						.max_fraction = 1.0f,
						.can_encroach = false
				};

				const auto output = ShapeCast::test(input, static_circle);
				expect(output.has_value() == "hit"_b);
			};
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.distance"> distance = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Vec2;
		using box2dpp::Rotation;
		using box2dpp::Transform;
		using box2dpp::ShapeProxy;
		using box2dpp::DistanceInput;
		using box2dpp::Distance;

		"circle_vs_circle"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			"Separated circles"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle circle_b{.center = {.x = 4, .y = 0}, .radius = 1};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// point_a=(1,0), point_b=(3,0), normal=(1,0), distance=2
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -3.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 2.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Touching circles"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle circle_b{.center = {.x = 2, .y = 0}, .radius = 1};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// point_a=(1,0), point_b=(1,0), normal=(1,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Overlapping circles"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 2};
				constexpr Circle circle_b{.center = {.x = 1, .y = 0}, .radius = 2};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// point_a=(2,0), point_b=(-1,0), normal=(1,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -2.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Zero radius circles (points)"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 0};
				constexpr Circle circle_b{.center = {.x = 3, .y = 4}, .radius = 0};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// point_a=(0,0), point_b=(3,4), normal=(0.6,0.8), distance=5
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -3.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + -4.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -.6f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + -.8f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 5.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"polygon_vs_polygon"_test = [] noexcept -> void
		{
			using box2dpp::Polygon;

			"Separated boxes"_test = [] noexcept -> void
			{
				const auto box_a = Polygon::make_box(1.0f, 0.5f);
				const auto box_b = Polygon::make_box(1.0f, 0.5f);
				constexpr Transform transform_a{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};
				constexpr Transform transform_b{.point = {.x = 3, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(box_a),
						.proxy_b = ShapeProxy::from(box_b),
						.transform_a = transform_a,
						.transform_b = transform_b,
						.use_radii = false
				};

				// point_a=(1,-0.5), point_b=(2,-0.5), normal=(1,0), distance=1
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + .5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -2.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + .5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Rotated overlapping boxes"_test = [] noexcept -> void
			{
				const auto box_a = Polygon::make_box(1.0f, 1.0f);
				const auto box_b = Polygon::make_box(1.0f, 1.0f);
				constexpr Transform transform_a{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};
				const Transform transform_b{.point = {.x = 0.5f, .y = 0}, .rotation = Rotation::from(0.785398f)}; // 45 degrees

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(box_a),
						.proxy_b = ShapeProxy::from(box_b),
						.transform_a = transform_a,
						.transform_b = transform_b,
						.use_radii = false
				};

				// normal=(0,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Edge-to-edge contact"_test = [] noexcept -> void
			{
				const auto box_a = Polygon::make_box(1.0f, 1.0f);
				const auto box_b = Polygon::make_box(1.0f, 1.0f);
				constexpr Transform transform_a{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};
				constexpr Transform transform_b{.point = {.x = 2, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(box_a),
						.proxy_b = ShapeProxy::from(box_b),
						.transform_a = transform_a,
						.transform_b = transform_b,
						.use_radii = false
				};

				// point_a=(1,-1), point_b=(1,-1), normal=(0,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Degenerate polygon (line segment)"_test = [] noexcept -> void
			{
				// Two-point polygon (line segment)
				constexpr std::array<Vec2, 2> points{{{.x = -1, .y = 0}, {.x = 1, .y = 0}}};
				constexpr Transform transform_a{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};
				constexpr Transform transform_b{.point = {.x = 0, .y = 2}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(points, 0),
						.proxy_b = ShapeProxy::from(points, 0),
						.transform_a = transform_a,
						.transform_b = transform_b,
						.use_radii = false
				};

				// point_a=(-1,0), point_b=(-1,2), normal=(0,1), distance=2
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + -2.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 2.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"edge_cases"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			"Same shape"_test = [] noexcept -> void
			{
				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle),
						.proxy_b = ShapeProxy::from(circle),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// normal=(0,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.normal.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Extremely close shapes"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle circle_b{.center = {.x = 2 + 1e-7f, .y = 0}, .radius = 1}; // Just barely touching
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				// point_a=(1,0), point_b=(1,0), normal=(1,0), distance=0
				const auto distance = Distance::compute(input);

				expect(std::abs(distance.point_a.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_a.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.point_b.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.x + -1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(distance.distance - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"Large shapes far apart"_test = [] noexcept -> void
			{
				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 100};
				constexpr Circle circle_b{.center = {.x = 1000, .y = 1000}, .radius = 100};
				constexpr Transform transform{.point = {.x = 0, .y = 0}, .rotation = Rotation::identity};

				// distance between centers minus sum of radii
				const auto expected = Vec2{.x = 1000, .y = 1000}.length() - 200;

				const DistanceInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.transform_a = transform,
						.transform_b = transform,
						.use_radii = true
				};

				const auto distance = Distance::compute(input);

				expect(std::abs(distance.distance - expected) <= value(std::numeric_limits<float>::epsilon()));
			};
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.time_of_impact"> time_of_impact = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Vec2;
		using box2dpp::Rotation;
		using box2dpp::Transform;
		using box2dpp::ShapeProxy;
		using box2dpp::Sweep;
		using box2dpp::TOIInput;
		using box2dpp::TOIState;
		using box2dpp::TOI;

		"linear_motion"_test = [] noexcept -> void
		{
			"Two circles moving toward each other"_test = [] noexcept -> void
			{
				using box2dpp::Circle;

				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 1};
				constexpr Circle circle_b{.center = {.x = 10, .y = 0}, .radius = 1};

				// Circle A moves right, Circle B moves left
				constexpr Sweep sweep_a
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 0, .y = 0},
						.c2 = {.x = 5, .y = 0}, // Move 5 units right
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};
				constexpr Sweep sweep_b
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 10, .y = 0},
						.c2 = {.x = 5, .y = 0}, // Move 5 units left
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};

				const TOIInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.sweep_a = sweep_a,
						.sweep_b = sweep_b,
						.max_fraction = 1.0f
				};

				const auto output = TOI::compute(input);

				expect(output.state == value(TOIState::SEPARATED));
			};

			"Circle misses moving box"_test = [] noexcept -> void
			{
				using box2dpp::Circle;
				using box2dpp::Polygon;

				constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
				const auto box = Polygon::make_box(1.0f, 1.0f);

				// Circle moves right, box moves up
				constexpr Sweep sweep_a
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 0, .y = 0},
						.c2 = {.x = 10, .y = 0},
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};
				constexpr Sweep sweep_b
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 5, .y = 0},
						.c2 = {.x = 5, .y = 10}, // Move up
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};

				const TOIInput input
				{
						.proxy_a = ShapeProxy::from(circle),
						.proxy_b = ShapeProxy::from(box),
						.sweep_a = sweep_a,
						.sweep_b = sweep_b,
						.max_fraction = 1.0f
				};

				const auto output = TOI::compute(input);

				expect(output.state == TOIState::SEPARATED);
			};

			"Initially overlapping"_test = [] noexcept -> void
			{
				using box2dpp::Circle;

				constexpr Circle circle_a{.center = {.x = 0, .y = 0}, .radius = 2};
				constexpr Circle circle_b{.center = {.x = 1, .y = 0}, .radius = 2};

				// Both move apart
				constexpr Sweep sweep_a
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 0, .y = 0},
						.c2 = {.x = -5, .y = 0}, // Move left
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};
				constexpr Sweep sweep_b
				{
						.local_center = {.x = 0, .y = 0},
						.c1 = {.x = 1, .y = 0},
						.c2 = {.x = 6, .y = 0}, // Move right
						.q1 = Rotation::identity,
						.q2 = Rotation::identity
				};

				const TOIInput input
				{
						.proxy_a = ShapeProxy::from(circle_a),
						.proxy_b = ShapeProxy::from(circle_b),
						.sweep_a = sweep_a,
						.sweep_b = sweep_b,
						.max_fraction = 1.0f
				};

				// state=HIT, point=(1,0), normal=(1,0), fraction=0
				auto output = TOI::compute(input);

				expect(output.state == value(TOIState::HIT));
			};
		};
	};
}
