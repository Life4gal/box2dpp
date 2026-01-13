#include <box2dpp/math/transform.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"math.transform"> _ = [] noexcept -> void
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

				expect(iv.x - two.x < value(std::numeric_limits<float>::epsilon() * 8.f));
				expect(iv.y - two.y < value(std::numeric_limits<float>::epsilon() * 8.f));
			};

			"multiply"_test = [&] noexcept -> void
			{
				const auto transform = transform2.multiply(transform1);

				const auto v = transform2.transform(transform1.transform(two));
				const auto u = transform.transform(two);

				expect(u.x - v.x < value(std::numeric_limits<float>::epsilon() * 10.f));
				expect(u.y - v.y < value(std::numeric_limits<float>::epsilon() * 10.f));
			};
		};
	};
}
