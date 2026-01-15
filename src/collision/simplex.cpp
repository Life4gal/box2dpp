// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/simplex.hpp>

#include <algorithm>
#include <ranges>

#include <box2dpp/collision/cast.hpp>

namespace box2dpp
{
	namespace
	{
		[[nodiscard]] auto weight2(const float a1, const Vec2& w1, const float a2, const Vec2& w2) noexcept -> Vec2
		{
			return a1 * w1 + a2 * w2;
		}

		[[nodiscard]] auto weight3(const float a1, const Vec2& w1, const float a2, const Vec2& w2, const float a3, const Vec2& w3) noexcept -> Vec2
		{
			return a1 * w1 + a2 * w2 + a3 * w3;
		}
	}

	auto SimplexCache::create(const Simplex& simplex) noexcept -> SimplexCache
	{
		BPP_ASSERT(simplex.count < std::numeric_limits<std::uint16_t>::max());

		SimplexCache result{.count = static_cast<std::uint16_t>(simplex.count), .index_a = {}, .index_b = {}};
		for (std::uint32_t i = 0; i < simplex.count; ++i)
		{
			const auto& vertex = simplex.vertices[i];

			BPP_ASSERT(vertex.index_a < std::numeric_limits<std::uint8_t>::max());
			BPP_ASSERT(vertex.index_b < std::numeric_limits<std::uint8_t>::max());

			result.index_a[i] = static_cast<std::uint8_t>(vertex.index_a);
			result.index_b[i] = static_cast<std::uint8_t>(vertex.index_b);
		}

		return result;
	}

	auto Simplex::create(const SimplexCache& cache, const ShapeProxy& proxy_a, const ShapeProxy& proxy_b) noexcept -> Simplex
	{
		BPP_ASSERT(cache.count <= 3);

		Simplex result{.vertices = {}, .count = cache.count};

		if (cache.count == 0)
		{
			result.count = 1;
			result.vertices[0] =
			{
					.w_a = proxy_a.points[0],
					.w_b = proxy_b.points[0],
					.w = proxy_a.points[0] - proxy_b.points[0],
					.a = 1.f,
					.index_a = 0,
					.index_b = 0
			};
		}
		else
		{
			for (auto [index, vertex]: std::views::enumerate(result.vertices))
			{
				vertex.index_a = cache.index_a[index];
				vertex.index_b = cache.index_b[index];
				vertex.w_a = proxy_a.points[vertex.index_a];
				vertex.w_b = proxy_b.points[vertex.index_b];
				vertex.w = vertex.w_a - vertex.w_b;

				// invalid
				vertex.a = -1.f;
			}
		}

		return result;
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
			// a2 <= 0, so we clamp it to 0
			vertices[0].a = 1.f;
			count = 1;

			return -w1;
		}

		// w2 region
		const auto d12_1 = w2.dot(e12);
		if (d12_1 <= 0)
		{
			// a1 <= 0, so we clamp it to 0
			vertices[1].a = 1.f;
			vertices[0] = vertices[1];
			count = 1;

			return -w2;
		}

		// Must be in e12 region.
		const auto inv_d12 = 1.f / (d12_1 + d12_2);
		vertices[0].a = d12_1 * inv_d12;
		vertices[1].a = d12_2 * inv_d12;
		count = 2;

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
			vertices[0].a = 1.f;
			count = 1;

			return -w1;
		}

		// e12
		if (d12_1 > 0 and d12_2 > 0 and d123_3 <= 0)
		{
			const auto inv_d12 = 1.f / (d12_1 + d12_2);

			vertices[0].a = d12_1 * inv_d12;
			vertices[1].a = d12_2 * inv_d12;
			count = 2;

			return cross((w1 + w2).cross(e12), e12);
		}

		// e13
		if (d13_1 > 0 and d13_2 > 0 and d123_2 <= 0)
		{
			const auto inv_d13 = 1.f / (d13_1 + d13_2);

			vertices[0].a = d13_1 * inv_d13;
			vertices[2].a = d13_2 * inv_d13;
			vertices[1] = vertices[2];
			count = 2;

			return cross((w1 + w3).cross(e13), e13);
		}

		// w2 region
		if (d12_1 <= 0 and d23_2 <= 0)
		{
			vertices[1].a = 1.f;
			vertices[0] = vertices[1];
			count = 1;

			return -w2;
		}

		// w3 region
		if (d13_1 <= 0 and d23_1 <= 0)
		{
			vertices[2].a = 1.f;
			vertices[0] = vertices[2];
			count = 1;

			return -w3;
		}

		// e23
		if (d23_1 > 0 and d23_2 > 0 and d123_1 <= 0)
		{
			const auto inv_d23 = 1.f / (d23_1 + d23_2);

			vertices[1].a = d23_1 * inv_d23;
			vertices[2].a = d23_2 * inv_d23;
			vertices[0] = vertices[2];
			count = 2;

			return cross((w2 + w3).cross(e23), e23);
		}

		// Must be in triangle123
		const auto inv_d123 = 1.f / (d123_1 + d123_2 + d123_3);
		vertices[0].a = d123_1 * inv_d123;
		vertices[1].a = d123_2 * inv_d123;
		vertices[2].a = d123_3 * inv_d123;
		count = 3;

		// No search direction
		return Vec2::zero;
	}

	auto Simplex::compute_witness_points(Vec2& out_a, Vec2& out_b) const noexcept -> void
	{
		if (count == 1)
		{
			const auto& v1 = vertices[0];

			out_a = v1.w_a;
			out_b = v1.w_b;
			return;
		}

		if (count == 2)
		{
			const auto& v1 = vertices[0];
			const auto& v2 = vertices[1];

			out_a = weight2(v1.a, v1.w_a, v2.a, v2.w_a);
			out_b = weight2(v1.a, v1.w_b, v2.a, v2.w_b);
			return;
		}

		if (count == 3)
		{
			const auto& v1 = vertices[0];
			const auto& v2 = vertices[1];
			const auto& v3 = vertices[2];

			out_a = weight3(v1.a, v1.w_a, v2.a, v2.w_a, v3.a, v3.w_a);
			// todo why are these not equal?
			// out_b = weight3(v1.a, v1.w_b, v2.a, v2.w_b, v3.a, v3.w_b);
			out_b = out_a;
			return;
		}

		BPP_COMPILER_UNREACHABLE();
	}
}
