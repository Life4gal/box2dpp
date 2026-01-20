#include <box2dpp/shape.hpp>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"shape"> _ = [] noexcept -> void
	{
		using namespace ut;
		using box2dpp::Circle;
		using box2dpp::Capsule;
		using box2dpp::Polygon;
		using box2dpp::Segment;

		constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};
		constexpr Capsule capsule{.center1 = {.x = -1, .y = 0}, .center2 = {.x = 1, .y = 0}, .radius = 1};
		constexpr Segment segment{.point1 = {.x = 0, .y = 1}, .point2 = {.x = 0, .y = -1}};

		const auto box = Polygon::make_box(1, 1);

		"mass"_test = [&] noexcept -> void
		{
			using box2dpp::Vec2;
			using box2dpp::Hull;
			using box2dpp::MassData;

			"circle"_test = [&] noexcept -> void
			{
				const auto md = MassData::compute(circle, 1);

				expect(std::abs(md.mass - std::numbers::pi_v<float>) <= value(std::numeric_limits<float>::epsilon()));
				expect(md.center == value(circle.center));
				expect(std::abs(md.rotational_inertia - std::numbers::pi_v<float> * .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"capsule"_test = [&] noexcept -> void
			{
				constexpr auto radius = capsule.radius;
				const auto length = capsule.center1.distance(capsule.center2);

				const auto md = MassData::compute(capsule, 1);

				// Box that full contains capsule
				const auto r = Polygon::make_box(radius, radius + length * .5f);
				const auto mdr = MassData::compute(r, 1);

				// Approximate capsule using convex hull
				constexpr std::size_t n = 4;
				constexpr auto d = std::numbers::pi_v<float> / static_cast<float>(n - 1);
				Vec2 points[n * 2];

				std::ranges::generate(
					std::views::counted(points, n),
					[angle = -std::numbers::pi_v<float> * .5f]() mutable noexcept -> Vec2
					{
						const Vec2 point{.x = 1.f + std::cos(angle) * radius, .y = std::sin(angle) * radius};
						angle += d;
						return point;
					}
				);
				std::ranges::generate(
					std::views::counted(points + n, n),
					[angle = std::numbers::pi_v<float> * .5f]() mutable noexcept -> Vec2
					{
						const Vec2 point{.x = -1.f + std::cos(angle) * radius, .y = std::sin(angle) * radius};
						angle += d;
						return point;
					}
				);

				const auto hull = Hull::create({points, n * 2});
				const auto ac = Polygon::make(hull, 0);
				const auto ma = MassData::compute(ac, 1);

				expect(ma.mass < md.mass and md.mass < mdr.mass);
				expect(ma.rotational_inertia < md.rotational_inertia and md.rotational_inertia < mdr.rotational_inertia);
			};

			"polygon"_test = [&] noexcept -> void
			{
				const auto md = MassData::compute(box, 1);

				expect(std::abs(md.mass - 4.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.center.x) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.center.y) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.rotational_inertia - 8.f / 3.f) <= value(std::numeric_limits<float>::epsilon() * 2));
			};
		};

		"aabb"_test = [&] noexcept -> void
		{
			using box2dpp::Transform;
			using box2dpp::AABB;

			"circle"_test = [&] noexcept -> void
			{
				const auto aabb = AABB::compute(circle, Transform::identity);

				expect(std::abs(aabb.lower.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 2.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"polygon"_test = [&] noexcept -> void
			{
				const auto aabb = AABB::compute(box, Transform::identity);

				expect(std::abs(aabb.lower.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"segment"_test = [&] noexcept -> void
			{
				const auto aabb = AABB::compute(segment, Transform::identity);

				expect(std::abs(aabb.lower.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"in"_test = [&] noexcept -> void
		{
			using box2dpp::Vec2;

			constexpr Vec2 p1{.x = .5f, .y = .5f};
			constexpr Vec2 p2{.x = 4.f, .y = -4.f};

			"circle"_test = [&] noexcept -> void
			{
				expect(circle.in(p1) == "p1 in circle"_b);
				expect(circle.in(p2) != "p1 not in circle"_b);
			};

			"polygon"_test = [&] noexcept -> void
			{
				expect(box.in(p1) == "p1 in box"_b);
				expect(box.in(p2) != "p2 not in box"_b);
			};
		};

		"distance"_test = [&] noexcept -> void
		{
			using box2dpp::Vec2;

			using box2dpp::SegmentDistance;

			constexpr Vec2 p1{.x = -1, .y = -1};
			constexpr Vec2 q1{.x = -1, .y = 1};
			constexpr Vec2 p2{.x = 2, .y = 0};
			constexpr Vec2 q2{.x = 1, .y = 0};
			constexpr Segment s1{.point1 = p1, .point2 = q1};
			constexpr Segment s2{.point1 = p2, .point2 = q2};

			"segment"_test = [&] noexcept -> void
			{
				const auto result = SegmentDistance::compute(s1, s2);

				expect(std::abs(result.closest1.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.closest1.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.closest2.x - 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.closest2.y - 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.fraction1 - .5f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.fraction2 - 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(result.distance_squared - 4.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};
	};
}
