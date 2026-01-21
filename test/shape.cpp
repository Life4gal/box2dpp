#include <box2dpp/shape.hpp>

#include <random>

#include <prometheus/ut/unit_test.hpp>
#include <prometheus/version-core.hpp>

using namespace prometheus;

namespace
{
	PROMETHEUS_COMPILER_NO_DESTROY ut::suite<"shape"> _ = [] noexcept -> void
	{
		using namespace ut;

		"circle"_test = [] noexcept -> void
		{
			using box2dpp::Circle;

			constexpr Circle c{.center = {.x = 0, .y = 0}, .radius = 2};

			expect(c.valid() == "valid circle"_b);
			constexpr Circle invalid{.center = {.x = 0, .y = 0}, .radius = -1};
			expect(invalid.valid() != "invalid circle with negative radius"_b);

			expect(c.in({.x = 2, .y = 0}) == "point on boundary"_b);
			expect(c.in({.x = 0, .y = 2}) == "point on boundary"_b);
			expect(c.in({.x = 1, .y = 1}) == "point inside"_b);
			expect(c.in({.x = 3, .y = 0}) != "point outside"_b);

			expect(std::abs(c.diameter() - 4.0f) < value(1e-6f));
			expect(std::abs(c.area() - 4.0f * std::numbers::pi_v<float>) < value(1e-6f));
			expect(std::abs(c.circumference() - 4.0f * std::numbers::pi_v<float>) < value(1e-6f));
		};

		"capsule"_test = [] noexcept -> void
		{
			using box2dpp::Capsule;

			constexpr Capsule c{.center1 = {.x = -2, .y = 0}, .center2 = {.x = 2, .y = 0}, .radius = 1};

			expect(c.valid() == "valid capsule"_b);
			constexpr Capsule invalid{.center1 = {.x = 0, .y = 0}, .center2 = {.x = 1, .y = 0}, .radius = -0.5f};
			expect(invalid.valid() != "invalid capsule"_b);

			expect(c.in({.x = -2, .y = 1}) == "point on top cap"_b);
			expect(c.in({.x = 2, .y = 1}) == "point on bottom cap"_b);
			expect(c.in({.x = 0, .y = 0.5f}) == "point on side"_b);
			expect(!c.in({.x = 0, .y = 2}) == "point outside"_b);

			constexpr Capsule degenerate{.center1 = {.x = 0, .y = 0}, .center2 = {.x = 0, .y = 0}, .radius = 1};
			expect(degenerate.in({.x = 0.5f, .y = 0.5f}) == "point in degenerate capsule"_b);

			expect(std::abs(c.length() - 4.0f) < value(1e-6f));
			expect(c.direction().dot({.x = 1, .y = 0}) > value(0.99f));
		};

		"segment"_test = [] noexcept -> void
		{
			using box2dpp::Segment;

			constexpr Segment s{.point1 = {.x = 0, .y = 0}, .point2 = {.x = 10, .y = 0}};

			const auto closest = s.closest_point({.x = 5, .y = 3});
			expect(std::abs(closest.x - 5) < value(1e-6f));
			expect(std::abs(closest.y - 0) < value(1e-6f));

			expect(std::abs(s.project({.x = 5, .y = 3}) - 0.5f) < value(1e-6f));
			expect(std::abs(s.project({.x = -1, .y = 0}) - 0.0f) < value(1e-6f));
			expect(std::abs(s.project({.x = 11, .y = 0}) - 1.0f) < value(1e-6f));

			expect(std::abs(s.distance_squared_to({.x = 5, .y = 3}) - 9) < value(1e-6f));

			expect(std::abs(s.direction().x - 1) < value(1e-6f));
			expect(std::abs(s.direction().y - 0) < value(1e-6f));

			const auto mid = s.midpoint();
			expect(std::abs(mid.x - 5) < value(1e-6f));
			expect(std::abs(mid.y - 0) < value(1e-6f));
		};

		"hull_creation"_test = [] noexcept -> void
		{
			using box2dpp::Vec2;
			using box2dpp::Hull;

			constexpr std::array<Vec2, 8> points
			{{
					{.x = 0, .y = 0},
					{.x = 1, .y = 0},
					{.x = 0, .y = 1},
					{.x = 0.5f, .y = 0.5f},
					{.x = -1, .y = 0},
					{.x = 0, .y = -1},
					{.x = 0.1f, .y = 0.1f},
					{.x = 0.1001f, .y = 0.1001f},
			}};

			const auto hull = Hull::create(points);
			expect(hull.valid() == "hull should be valid"_b);
			expect(hull.count >= 4_u and hull.count <= 6_u) << "hull should have 4-6 vertices";

			const auto empty_hull = Hull::create({});
			expect(empty_hull.count == 0_u) << "empty input should produce empty hull";

			constexpr std::array<Vec2, 3> collinear
			{{
					{.x = 0, .y = 0},
					{.x = 1, .y = 1},
					{.x = 2, .y = 2},
			}};

			const auto collinear_hull = Hull::create(collinear);
			expect(collinear_hull.valid() != "collinear points should not produce valid hull"_b);
		};

		"polygon_creation"_test = [] noexcept -> void
		{
			using box2dpp::Rotation;
			using box2dpp::Polygon;

			const auto square = Polygon::make_square(2);
			expect(square.count == 4_u);
			expect(square.radius == 0_f);

			const auto rounded_box = Polygon::make_box(1, 2, 0.2f);
			expect(rounded_box.count == 4_u);
			expect(rounded_box.radius == 0.2_f);

			const auto rotated_box = Polygon::make_box(1, 1, {.x = 2, .y = 3}, Rotation::from(std::numbers::pi_v<float> / 4));
			expect(rotated_box.count == 4_u);

			const auto box = Polygon::make_box(2, 3);
			expect(box.in({.x = 0, .y = 0}) == "center point"_b);
			expect(box.in({.x = 2, .y = 0}) == "right edge"_b);
			expect(box.in({.x = 2.1f, .y = 0}) != "outside right"_b);
			expect(box.in({.x = 0, .y = 3.1f}) != "outside top"_b);
		};

		"mass_data"_test = [] noexcept -> void
		{
			using box2dpp::MassData;

			"circle"_test = [&] noexcept -> void
			{
				using box2dpp::Circle;

				constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};

				const auto md = MassData::compute(circle, 1);

				expect(std::abs(md.mass - std::numbers::pi_v<float>) <= value(std::numeric_limits<float>::epsilon()));
				expect(md.center == value(circle.center));
				expect(std::abs(md.rotational_inertia - std::numbers::pi_v<float> * .5f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"capsule"_test = [&] noexcept -> void
			{
				using box2dpp::Vec2;
				using box2dpp::Capsule;
				using box2dpp::Hull;
				using box2dpp::Polygon;

				constexpr Capsule capsule{.center1 = {.x = -1, .y = 0}, .center2 = {.x = 1, .y = 0}, .radius = 1};

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
				using box2dpp::Polygon;

				const auto box = Polygon::make_box(1, 1);

				const auto md = MassData::compute(box, 1);

				expect(std::abs(md.mass - 4.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.center.x) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.center.y) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(md.rotational_inertia - 8.f / 3.f) <= value(std::numeric_limits<float>::epsilon() * 2));
			};
		};

		"mass_data_edge_cases"_test = [] noexcept -> void
		{
			using box2dpp::Circle;
			using box2dpp::MassData;

			constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};

			constexpr Circle tiny_circle{.center = {.x = 0, .y = 0}, .radius = 0.001f};
			const auto md_tiny = MassData::compute(tiny_circle, 1);
			expect(md_tiny.valid() == "tiny circle should have valid mass data"_b);

			const auto md_high_density = MassData::compute(circle, 1000);
			expect(md_high_density.mass > 0_f);
			expect(md_high_density.valid());
		};

		"aabb"_test = [] noexcept -> void
		{
			using box2dpp::Transform;
			using box2dpp::AABB;

			"circle"_test = [&] noexcept -> void
			{
				using box2dpp::Circle;

				constexpr Circle circle{.center = {.x = 1, .y = 0}, .radius = 1};

				const auto aabb = AABB::compute(circle, Transform::identity);

				expect(std::abs(aabb.lower.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 2.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"polygon"_test = [&] noexcept -> void
			{
				using box2dpp::Polygon;

				const auto box = Polygon::make_box(1, 1);

				const auto aabb = AABB::compute(box, Transform::identity);

				expect(std::abs(aabb.lower.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};

			"segment"_test = [&] noexcept -> void
			{
				using box2dpp::Segment;

				constexpr Segment segment{.point1 = {.x = 0, .y = 1}, .point2 = {.x = 0, .y = -1}};

				const auto aabb = AABB::compute(segment, Transform::identity);

				expect(std::abs(aabb.lower.x + 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.lower.y + 1.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.x - 0.f) <= value(std::numeric_limits<float>::epsilon()));
				expect(std::abs(aabb.upper.y - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			};
		};

		"aabb_comprehensive"_test = [] noexcept -> void
		{
			using box2dpp::AABB;

			constexpr AABB aabb1{.lower = {.x = 0, .y = 0}, .upper = {.x = 2, .y = 2}};
			constexpr AABB aabb2{.lower = {.x = 1, .y = 1}, .upper = {.x = 3, .y = 3}};

			expect(aabb1.valid() == "aabb1 should be valid"_b);
			constexpr AABB invalid{.lower = {.x = 3, .y = 3}, .upper = {.x = 1, .y = 1}};
			expect(invalid.valid() != "invalid aabb should not be valid"_b);

			expect(aabb1.overlaps(aabb2) == "aabb1 and aabb2 overlap"_b);

			constexpr AABB aabb3{.lower = {.x = 0.5f, .y = 0.5f}, .upper = {.x = 1.5f, .y = 1.5f}};

			expect(aabb1.contains(aabb3) == "aabb1 contains aabb3"_b);
			expect(aabb3.contains(aabb1) != "aabb3 does not contain aabb1"_b);

			expect(aabb1.contains({.x = 1, .y = 1}) == "point inside aabb1"_b);
			expect(aabb1.contains({.x = 3, .y = 3}) != "point outside aabb1"_b);

			const auto union_aabb = aabb1.combination_max(aabb2);
			expect(std::abs(union_aabb.lower.x - 0) < value(1e-6f));
			expect(std::abs(union_aabb.lower.y - 0) < value(1e-6f));
			expect(std::abs(union_aabb.upper.x - 3) < value(1e-6f));
			expect(std::abs(union_aabb.upper.y - 3) < value(1e-6f));

			const auto intersect_aabb = aabb1.combination_min(aabb2);
			expect(std::abs(intersect_aabb.lower.x - 1) < value(1e-6f));
			expect(std::abs(intersect_aabb.lower.y - 1) < value(1e-6f));
			expect(std::abs(intersect_aabb.upper.x - 2) < value(1e-6f));
			expect(std::abs(intersect_aabb.upper.y - 2) < value(1e-6f));

			AABB expanding = aabb1;
			const bool enlarged = expanding.enlarge({.x = 4, .y = 4});
			expect(enlarged == "aabb should be enlarged"_b);
			expect(expanding.contains({.x = 4, .y = 4}) == "expanded aabb should contain point"_b);

			expect(std::abs(aabb1.width() - 2) < value(1e-6f));
			expect(std::abs(aabb1.height() - 2) < value(1e-6f));
			expect(std::abs(aabb1.area() - 4) < value(1e-6f));
			expect(std::abs(aabb1.perimeter() - 8) < value(1e-6f));

			const auto center = aabb1.center();
			expect(std::abs(center.x - 1) < value(1e-6f));
			expect(std::abs(center.y - 1) < value(1e-6f));

			const auto extents = aabb1.extents();
			expect(std::abs(extents.x - 1) < value(1e-6f));
			expect(std::abs(extents.y - 1) < value(1e-6f));
		};

		"segment_distance"_test = [] noexcept -> void
		{
			using box2dpp::Vec2;
			using box2dpp::Segment;
			using box2dpp::SegmentDistance;

			constexpr Vec2 p1{.x = -1, .y = -1};
			constexpr Vec2 q1{.x = -1, .y = 1};
			constexpr Vec2 p2{.x = 2, .y = 0};
			constexpr Vec2 q2{.x = 1, .y = 0};
			constexpr Segment s1{.point1 = p1, .point2 = q1};
			constexpr Segment s2{.point1 = p2, .point2 = q2};

			const auto result = SegmentDistance::compute(s1, s2);

			expect(std::abs(result.closest1.x + 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.closest1.y + 0.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.closest2.x - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.closest2.y - 0.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.fraction1 - .5f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.fraction2 - 1.f) <= value(std::numeric_limits<float>::epsilon()));
			expect(std::abs(result.distance_squared - 4.f) <= value(std::numeric_limits<float>::epsilon()));
		};

		"segment_distance_edge_cases"_test = [] noexcept -> void
		{
			using box2dpp::Segment;
			using box2dpp::SegmentDistance;

			constexpr Segment s1{.point1 = {.x = 0, .y = 0}, .point2 = {.x = 2, .y = 0}};
			constexpr Segment s2{.point1 = {.x = 0, .y = 1}, .point2 = {.x = 2, .y = 1}};
			const auto dist1 = SegmentDistance::compute(s1, s2);

			expect(dist1.valid() == "distance result should be valid"_b);
			expect(std::abs(dist1.distance() - 1) < value(1e-6f)) << "distance should be sqrt(distance_squared)";
			expect(std::abs(dist1.distance_squared - 1) < value(1e-6f));
			expect(std::abs(dist1.closest1.x - 0) < value(1e-6f));

			constexpr Segment s3{.point1 = {.x = -1, .y = -1}, .point2 = {.x = 1, .y = 1}};
			constexpr Segment s4{.point1 = {.x = -1, .y = 1}, .point2 = {.x = 1, .y = -1}};
			const auto dist2 = SegmentDistance::compute(s3, s4);
			expect(dist2.distance_squared < value(1e-6f)) << "intersecting segments should have zero distance";

			constexpr Segment point_segment{.point1 = {.x = 0, .y = 0}, .point2 = {.x = 0, .y = 0}};
			const auto dist3 = SegmentDistance::compute(s1, point_segment);
			expect(std::abs(dist3.distance_squared - 0) < value(1e-6f));

			constexpr Segment s5{.point1 = {.x = 0, .y = 0}, .point2 = {.x = 0, .y = 1}};
			constexpr Segment s6{.point1 = {.x = 2, .y = 0}, .point2 = {.x = 2, .y = 1}};
			const auto dist4 = SegmentDistance::compute(s5, s6);
			expect(std::abs(dist4.distance_squared - 4) < value(1e-6f));
			expect(dist4.fraction1 == 0_f or dist4.fraction1 == 1_f) << "closest point should be at endpoint";
		};

		"transform_operations"_test = [] noexcept -> void
		{
			using box2dpp::Rotation;
			using box2dpp::Transform;
			using box2dpp::Circle;
			using box2dpp::Polygon;
			using box2dpp::AABB;

			const Transform transform
			{
					.point = {.x = 2, .y = 3},
					.rotation = Rotation::from(std::numbers::pi_v<float> / 2)
			};

			constexpr Circle circle{.center = {.x = 0, .y = 0}, .radius = 1};
			const auto circle_aabb = AABB::compute(circle, transform);

			expect(std::abs(circle_aabb.width() - 2) < value(1e-6f));
			expect(std::abs(circle_aabb.height() - 2) < value(1e-6f));

			const auto polygon = Polygon::make_box(1, 2);
			const auto transformed_polygon = Polygon::transform(transform, polygon);
			expect(transformed_polygon.count == value(polygon.count));

			const auto original_box = Polygon::make_box(1, 1);
			constexpr Transform translate{.point = {.x = 5, .y = 5}, .rotation = Rotation::identity};
			const auto translated_box = Polygon::transform(translate, original_box);
			expect(translated_box.in({.x = 5, .y = 5}) == "center should be at (5,5)"_b);
			expect(translated_box.in({.x = 0, .y = 0}) != "original center should not be inside"_b);
		};

		"randomized_tests"_test = [] noexcept -> void
		{
			using box2dpp::Vec2;
			using box2dpp::Circle;
			using box2dpp::AABB;

			std::mt19937 engine{42};
			std::uniform_real_distribution dist{-10.f, 10.f};
			std::uniform_real_distribution radius_dist{0.1f, 5.0f};

			for (int i = 0; i < 100; ++i)
			{
				const Circle random_circle{.center = {.x = dist(engine), .y = dist(engine)}, .radius = radius_dist(engine)};

				expect((random_circle.valid() == (random_circle.radius > 0)) == "circle validity check"_b);

				const Vec2 random_point{.x = dist(engine), .y = dist(engine)};
				const bool in_circle = random_circle.in(random_point);
				const float distance_sq = random_circle.center.distance_squared(random_point);
				const bool expected = distance_sq <= random_circle.radius * random_circle.radius;
				expect((in_circle == expected) == "circle point containment should match distance calculation"_b);

				const AABB random_aabb
				{
						.lower = {.x = dist(engine), .y = dist(engine)},
						.upper = {.x = dist(engine) + 1, .y = dist(engine) + 1}
				};

				if (random_aabb.valid())
				{
					const auto center = random_aabb.center();
					expect(random_aabb.contains(center) == "aabb should contain its center"_b);

					AABB copy = random_aabb;
					const Vec2 far_point{.x = dist(engine) + 20, .y = dist(engine) + 20};
					copy.enlarge(far_point);
					expect(copy.contains(far_point) == "enlarged aabb should contain the point"_b);
				}
			}
		};

		"error_handling"_test = [] noexcept -> void
		{
			using box2dpp::Circle;
			using box2dpp::AABB;
			using box2dpp::MassData;

			constexpr float nan = std::numeric_limits<float>::quiet_NaN();
			constexpr float inf = std::numeric_limits<float>::infinity();

			constexpr Circle nan_circle{.center = {.x = nan, .y = 0}, .radius = 1};
			expect(nan_circle.valid() != "circle with NaN center should not be valid"_b);

			constexpr Circle inf_circle{.center = {.x = 0, .y = 0}, .radius = inf};
			expect(inf_circle.valid() != "circle with infinite radius should not be valid"_b);

			constexpr AABB nan_aabb{.lower = {.x = 0, .y = 0}, .upper = {.x = nan, .y = 1}};
			expect(nan_aabb.valid() != "aabb with NaN coordinate should not be valid"_b);

			constexpr MassData invalid_mass{.mass = -1, .center = {.x = 0, .y = 0}, .rotational_inertia = 1};
			expect(invalid_mass.valid() != "mass data with negative mass should not be valid"_b) << "";
		};
	};
}
