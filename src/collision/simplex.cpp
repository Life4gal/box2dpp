// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/simplex.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/utility/value_cast.hpp>

namespace box2dpp
{
	namespace
	{
		[[nodiscard]] auto type_cast(const SimplexCache::Type type) noexcept -> Simplex::Type
		{
			BPP_ASSERT(type != SimplexCache::Type::UNINITIALIZED);

			return static_cast<Simplex::Type>(type);
		}

		[[nodiscard]] auto type_cast(const Simplex::Type type) noexcept -> SimplexCache::Type
		{
			return static_cast<SimplexCache::Type>(type);
		}
	}

	auto Simplex::create(const SimplexCache& cache, const ShapeProxy& proxy_a, const ShapeProxy& proxy_b) noexcept -> Simplex
	{
		if (cache.type == SimplexCache::Type::UNINITIALIZED)
		{
			Simplex result{.vertices = {}, .type = Type::POINT};

			result.vertices[0] =
			{
					.w_a = proxy_a.points[0],
					.w_b = proxy_b.points[0],
					.w = proxy_a.points[0] - proxy_b.points[0],
					.weight = 1.f,
					.index_a = 0,
					.index_b = 0
			};

			return result;
		}

		Simplex result{.vertices = {}, .type = type_cast(cache.type)};
		for (auto [index, vertex]: std::views::enumerate(result.vertices))
		{
			vertex.index_a = cache.index_a[index];
			vertex.index_b = cache.index_b[index];
			vertex.w_a = proxy_a.points[vertex.index_a];
			vertex.w_b = proxy_b.points[vertex.index_b];
			vertex.w = vertex.w_a - vertex.w_b;

			// invalid
			vertex.weight = -1.f;
		}

		return result;
	}

	auto Simplex::cache() const noexcept -> SimplexCache
	{
		SimplexCache result{.type = type_cast(type), .index_a = {}, .index_b = {}};
		for (const auto [index, vertex]:
		     vertices |
		     std::views::take(static_cast<std::ptrdiff_t>(type)) |
		     std::views::enumerate
		)
		{
			result.index_a[index] = vertex.index_a;
			result.index_b[index] = vertex.index_b;
		}

		return result;
	}

	auto Simplex::solve1() const noexcept -> Vec2
	{
		const auto w = vertices[0].w;

		return -w;
	}

	auto Simplex::solve2() noexcept -> Vec2
	{
		const auto w1 = vertices[0].w;
		const auto w2 = vertices[1].w;
		const auto e12 = w2 - w1;

		// w1 region
		const auto d12_2 = -w1.dot(e12);
		if (d12_2 <= 0)
		{
			// Origin is closest to w1
			vertices[0].weight = 1.f;
			type = Type::POINT;

			// Direction from w1 to origin
			return -w1;
		}

		// w2 region
		const auto d12_1 = w2.dot(e12);
		if (d12_1 <= 0)
		{
			// Origin is closest to w2
			vertices[0] = vertices[1];
			vertices[0].weight = 1.f;
			type = Type::POINT;

			// Direction from w2 to origin
			return -w2;
		}

		// Origin is closest to the line segment between w1 and w2
		// Compute barycentric coordinates
		const float denominator = d12_1 + d12_2;
		BPP_ASSERT(denominator > 0.0f);

		const float inv_denominator = 1.0f / denominator;
		vertices[0].weight = d12_1 * inv_denominator;
		vertices[1].weight = d12_2 * inv_denominator;
		type = Type::LINE_SEGMENT;

		return cross((w1 + w2).cross(e12), e12);
	}

	auto Simplex::solve3() noexcept -> Vec2
	{
		const auto w1 = vertices[0].w;
		const auto w2 = vertices[1].w;
		const auto w3 = vertices[2].w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		const auto e12 = w2 - w1;
		const auto w1_e12 = w1.dot(e12);
		const auto w2_e12 = w2.dot(e12);
		const auto d12_1 = w2_e12;
		const auto d12_2 = -w1_e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		const auto e13 = w3 - w1;
		const auto w1_e13 = w1.dot(e13);
		const auto w3_e13 = w3.dot(e13);
		const auto d13_1 = w3_e13;
		const auto d13_2 = -w1_e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		const auto e23 = w3 - w2;
		const auto w2_e23 = w2.dot(e23);
		const auto w3_e23 = w3.dot(e23);
		const auto d23_1 = w3_e23;
		const auto d23_2 = -w2_e23;

		// Triangle123
		const auto n123 = e12.cross(e13);

		const auto d123_1 = n123 * w2.cross(w3);
		const auto d123_2 = n123 * w3.cross(w1);
		const auto d123_3 = n123 * w1.cross(w2);

		// w1 region
		if (d12_2 <= 0 and d13_2 <= 0)
		{
			vertices[0].weight = 1.f;
			type = Type::POINT;

			return -w1;
		}

		// e12
		if (d12_1 > 0 and d12_2 > 0 and d123_3 <= 0)
		{
			const auto inv_d12 = 1.f / (d12_1 + d12_2);

			vertices[0].weight = d12_1 * inv_d12;
			vertices[1].weight = d12_2 * inv_d12;
			type = Type::LINE_SEGMENT;

			return cross((w1 + w2).cross(e12), e12);
		}

		// e13
		if (d13_1 > 0 and d13_2 > 0 and d123_2 <= 0)
		{
			const auto inv_d13 = 1.f / (d13_1 + d13_2);

			vertices[0].weight = d13_1 * inv_d13;
			vertices[2].weight = d13_2 * inv_d13;
			vertices[1] = vertices[2];
			type = Type::LINE_SEGMENT;

			return cross((w1 + w3).cross(e13), e13);
		}

		// w2 region
		if (d12_1 <= 0 and d23_2 <= 0)
		{
			vertices[1].weight = 1.f;
			vertices[0] = vertices[1];
			type = Type::POINT;

			return -w2;
		}

		// w3 region
		if (d13_1 <= 0 and d23_1 <= 0)
		{
			vertices[2].weight = 1.f;
			vertices[0] = vertices[2];
			type = Type::POINT;

			return -w3;
		}

		// e23
		if (d23_1 > 0 and d23_2 > 0 and d123_1 <= 0)
		{
			const auto inv_d23 = 1.f / (d23_1 + d23_2);

			vertices[1].weight = d23_1 * inv_d23;
			vertices[2].weight = d23_2 * inv_d23;
			vertices[0] = vertices[2];
			type = Type::LINE_SEGMENT;

			return cross((w2 + w3).cross(e23), e23);
		}

		// Must be in triangle123
		const auto inv_d123 = 1.f / (d123_1 + d123_2 + d123_3);
		vertices[0].weight = d123_1 * inv_d123;
		vertices[1].weight = d123_2 * inv_d123;
		vertices[2].weight = d123_3 * inv_d123;
		type = Type::TRIANGLE;

		// No search direction
		return Vec2::zero;
	}

	auto Simplex::solve() noexcept -> Vec2
	{
		switch (type)
		{
			case Type::POINT:
			{
				return solve1();
			}
			case Type::LINE_SEGMENT:
			{
				return solve2();
			}
			case Type::TRIANGLE:
			{
				return solve3();
			}
		}

		BPP_COMPILER_UNREACHABLE();
	}

	auto Simplex::compute_closest_points() const noexcept -> std::pair<Vec2, Vec2>
	{
		switch (type)
		{
			// Point simplex
			case Type::POINT:
			{
				const auto& v1 = vertices[0];

				return {v1.w_a, v1.w_b};
			}
			// Line segment simplex
			case Type::LINE_SEGMENT:
			{
				const auto& v1 = vertices[0];
				const auto& v2 = vertices[1];

				// Closest point is convex combination of the two vertices
				return
				{
						v1.weight * v1.w_a + v2.weight * v2.w_a,
						v1.weight * v1.w_b + v2.weight * v2.w_b,
				};
			}
			// Triangle simplex (shapes are overlapping)
			// The closest points are not uniquely defined when shapes overlap
			// Use the barycentric coordinates already computed in solve3().
			case Type::TRIANGLE:
			{
				const auto& v1 = vertices[0];
				const auto& v2 = vertices[1];
				const auto& v3 = vertices[2];

				return
				{
						v1.weight * v1.w_a + v2.weight * v2.w_a + v3.weight * v3.w_a,
						v1.weight * v1.w_b + v2.weight * v2.w_b + v3.weight * v3.w_b,
				};
			}
		}

		BPP_COMPILER_UNREACHABLE();
	}
}
