#include <box2dpp/collision.hpp>
#include <box2dpp/shape.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.ray_cast"> ray_cast = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Circle;
		using box2dpp::Capsule;
		using box2dpp::Polygon;
		using box2dpp::Segment;

		using box2dpp::RayCastInput;
		using box2dpp::RayCast;

		constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};
		constexpr Segment segment{.point1 = {.x = 0, .y = 1}, .point2 = {.x = 0, .y = -1}};

		const auto box = Polygon::make_box(1, 1);

		constexpr RayCastInput input{.origin = {.x = -4, .y = 0}, .translation = {.x = 8, .y = 0}, .max_fraction = 1};

		"circle"_test = [&] noexcept -> void
		{
			const auto output = RayCast::test(input, circle);

			expect(output.hit == "circle hit"_b);
			expect(std::abs(output.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.fraction - .5f) <= std::numeric_limits<float>::epsilon());
		};

		"box"_test = [&] noexcept -> void
		{
			const auto output = RayCast::test(input, box);

			expect(output.hit == "box hit"_b);
			expect(std::abs(output.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.fraction - 3.f / 8.f) <= std::numeric_limits<float>::epsilon());
		};

		"segment"_test = [&] noexcept -> void
		{
			const auto output = RayCast::test(input, segment, true);

			expect(output.hit == "segment hit"_b);
			expect(std::abs(output.normal.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.normal.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(output.fraction - .5f) <= std::numeric_limits<float>::epsilon());
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.shape_cast"> shape_cast = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::Vec2;
		using box2dpp::Transform;

		using box2dpp::ShapeProxy;
		// using box2dpp::ShapeCastInput;
		using box2dpp::ShapeCastPairInput;
		using box2dpp::ShapeCast;

		constexpr Vec2 vas[]
		{
				{.x = -1, .y = -1},
				{.x = 1, .y = -1},
				{.x = 1, .y = 1},
				{.x = -1, .y = 1},
		};
		constexpr Vec2 vbs[]
		{
				{.x = 2, .y = -1},
				{.x = 2, .y = 1},
		};

		"pair"_test = [&] noexcept -> void
		{
			const ShapeCastPairInput pair_input
			{
					.proxy_a = ShapeProxy::create({vas, std::ranges::size(vas)}, 0),
					.proxy_b = ShapeProxy::create({vbs, std::ranges::size(vbs)}, 0),
					.transform_a = Transform::identity,
					.transform_b = Transform::identity,
					.translation_b = {.x = -2, .y = 0},
					.max_fraction = 1,
					.can_encroach = false
			};
			const auto output = ShapeCast::test(pair_input);

			expect(output.hit == "shape hit"_b);
			expect(std::abs(output.fraction - .5f) <= .005_f);
		};
	};

	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"collision.distance"> distance = [] noexcept -> void
	{
		using namespace ut;

		using box2dpp::DistanceInput;
		using box2dpp::DistanceOutput;

		"shape"_test = [&] noexcept -> void
		{
			using box2dpp::Vec2;
			using box2dpp::Transform;

			using box2dpp::ShapeProxy;

			constexpr Vec2 vas[]
			{
					{.x = -1, .y = -1},
					{.x = 1, .y = -1},
					{.x = 1, .y = 1},
					{.x = -1, .y = 1},
			};
			constexpr Vec2 vbs[]
			{
					{.x = 2, .y = -1},
					{.x = 2, .y = 1},
			};

			const DistanceInput input
			{
					.proxy_a = ShapeProxy::create({vas, std::ranges::size(vas)}, 0),
					.proxy_b = ShapeProxy::create({vbs, std::ranges::size(vbs)}, 0),
					.transform_a = Transform::identity,
					.transform_b = Transform::identity,
					.use_radii = false
			};
			const auto output = DistanceOutput::compute(input);

			expect(std::abs(output.distance - 1) <= value(std::numeric_limits<float>::epsilon()));
		};
	};
}
