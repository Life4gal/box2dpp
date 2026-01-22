// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <box2dpp/collision/time_of_impact.hpp>

#include <algorithm>

#include <box2dpp/collision/distance.hpp>

namespace box2dpp
{
	auto Sweep::valid() const noexcept -> bool
	{
		if (not local_center.valid())
		{
			return false;
		}

		if (not c1.valid() or not c2.valid())
		{
			return false;
		}

		if (not q1.valid() or not q2.valid())
		{
			return false;
		}

		return true;
	}

	auto Sweep::transform_of(const float t) const noexcept -> Transform
	{
		BPP_ASSERT(valid());

		const Rotation q{.cos = (1.f - t) * q1.cos + t * q2.cos, .sin = (1.f - t) * q1.sin + t * q2.sin};
		const auto q_normalized = q.normalize();

		auto p = (1.f - t) * c1 + t * c2;
		// Shift to origin
		p -= q_normalized.rotate(local_center);

		return {.point = p, .rotation = q_normalized};
	}

	auto Sweep::linear_velocity() const noexcept -> Vec2
	{
		BPP_ASSERT(valid());

		return c2 - c1;
	}

	auto Sweep::angular_displacement() const noexcept -> float
	{
		BPP_ASSERT(valid());

		return unwind_angle(q2.angle() - q1.angle());
	}

	auto Sweep::advance(const float fraction) const noexcept -> Sweep
	{
		BPP_ASSERT(valid());

		// Create new sweep starting from current advanced position
		const auto advanced = transform_of(fraction);
		const auto remaining_fraction = 1.0f - fraction;

		// Compute new end position for remaining motion
		const auto new_c1 = advanced.point + advanced.rotation.rotate(local_center);
		// Same linear motion
		const auto new_c2 = c1 + (c2 - c1);

		// Compute new end rotation
		const auto new_angle = advanced.rotation.angle() + angular_displacement() * remaining_fraction;
		const auto new_q2 = Rotation::from(new_angle);

		return {.local_center = local_center, .c1 = new_c1, .c2 = new_c2, .q1 = advanced.rotation, .q2 = new_q2};
	}

	auto TOIInput::valid() const noexcept -> bool
	{
		if (not proxy_a.valid() or not proxy_b.valid())
		{
			return false;
		}

		if (not sweep_a.valid() or not sweep_b.valid())
		{
			return false;
		}

		if (not box2dpp::valid(max_fraction) or max_fraction < 0 or max_fraction > 1)
		{
			return false;
		}

		return true;
	}

	namespace
	{
		/// Helper class for computing separation along an axis over time.
		/// Manages witness points and separation function evaluation.
		class SeparationSolver final // NOLINT(clang-diagnostic-padded)
		{
		public:
			using index_type = ShapeProxy::index_type;

			enum class AxisType : std::uint8_t
			{
				/// Point-to-point separation
				POINTS,

				/// Face of shape A vs point of shape B
				FACE_A,

				/// Face of shape B vs point of shape A
				FACE_B
			};

		private:
			std::reference_wrapper<const ShapeProxy> proxy_a_;
			std::reference_wrapper<const ShapeProxy> proxy_b_;
			Sweep sweep_a_;
			Sweep sweep_b_;

			/// Witness point in local coordinates (for face axes)
			Vec2 local_witness_;

			/// Separation axis in local coordinates
			Vec2 local_axis_;

			/// Type of separation axis
			AxisType axis_type_;

			/// Cached indices for witness points
			index_type cached_index_a_;
			index_type cached_index_b_;

			/// Whether axis direction should be flipped (for face axes)
			bool flip_axis_;

			/// Initialize from simplex cache
			void initialize_from_cache(
				const SimplexCache& cache,
				const Transform& transform_a,
				const Transform& transform_b
			) noexcept
			{
				const auto& proxy_a = proxy_a_.get();
				const auto& proxy_b = proxy_b_.get();

				if (cache.type == SimplexCache::Type::POINT)
				{
					// Point simplex: use vector between points as separation axis

					const auto index_a = cache.index_a[0];
					const auto index_b = cache.index_b[0];

					const auto local_point_a = proxy_a.points[index_a];
					const auto local_point_b = proxy_b.points[index_b];

					const auto point_a = transform_a.transform(local_point_a);
					const auto point_b = transform_b.transform(local_point_b);
					const auto axis = (point_b - point_a).normalize();

					local_axis_ = axis;
					local_witness_ = Vec2::zero;
					axis_type_ = AxisType::POINTS;
					flip_axis_ = false;

					cached_index_a_ = index_a;
					cached_index_b_ = index_b;
				}
				else if (cache.index_a[0] == cache.index_a[1])
				{
					// Two points on B, one on A -> face of B

					const auto index_a = cache.index_a[0];
					const auto index_b1 = cache.index_b[0];
					const auto index_b2 = cache.index_b[1];

					const auto local_point_b1 = proxy_b.points[index_b1];
					const auto local_point_b2 = proxy_b.points[index_b2];

					// Edge vector and normal
					const auto edge = local_point_b2 - local_point_b1;
					// Perpendicular (normal)
					local_axis_ = edge.cross(1).normalize();
					// Midpoint of edge
					local_witness_ = (local_point_b1 + local_point_b2) * .5f;

					// Check which side shape A is on
					const auto local_point_a = proxy_a.points[index_a];
					const auto point_a = transform_a.transform(local_point_a);
					const auto point_b = transform_b.transform(local_witness_);
					const auto normal_world = transform_b.rotation.rotate(local_axis_);

					const float sign = (point_a - point_b).dot(normal_world);
					flip_axis_ = sign < 0;

					axis_type_ = AxisType::FACE_B;
					cached_index_a_ = index_a;
					cached_index_b_ = ShapeProxy::invalid_index;
				}
				else
				{
					// Two points on A, one or two on B -> face of A

					const auto index_a1 = cache.index_a[0];
					const auto index_a2 = cache.index_a[1];
					const auto index_b = cache.index_b[0];

					const auto local_point_a1 = proxy_a.points[index_a1];
					const auto local_point_a2 = proxy_a.points[index_a2];

					// Edge vector and normal
					const auto edge = local_point_a2 - local_point_a1;
					// Perpendicular (normal)
					local_axis_ = edge.cross(1).normalize();
					// Midpoint of edge
					local_witness_ = (local_point_a1 + local_point_a2) * .5f;

					// Check which side shape B is on
					const auto local_point_b = proxy_b.points[index_b];
					const auto point_a = transform_a.transform(local_witness_);
					const auto point_b = transform_b.transform(local_point_b);
					const auto normal_world = transform_a.rotation.rotate(local_axis_);

					const float sign = (point_b - point_a).dot(normal_world);
					flip_axis_ = sign < 0;

					axis_type_ = AxisType::FACE_A;
					cached_index_a_ = ShapeProxy::invalid_index;
					cached_index_b_ = index_b;
				}
			}


			/// Evaluate separation for specific witness indices
			[[nodiscard]] auto evaluate_with_indices(
				const Transform& transform_a,
				const Transform& transform_b,
				const Vec2& normal,
				const index_type index_a,
				const index_type index_b
			) const noexcept -> float
			{
				const auto& proxy_a = proxy_a_.get();
				const auto& proxy_b = proxy_b_.get();

				switch (axis_type_)
				{
					case AxisType::POINTS:
					{
						BPP_ASSERT(index_a != ShapeProxy::invalid_index);
						BPP_ASSERT(index_b != ShapeProxy::invalid_index);

						const auto local_point_a = proxy_a.points[index_a];
						const auto local_point_b = proxy_b.points[index_b];

						const auto point_a = transform_a.transform(local_point_a);
						const auto point_b = transform_b.transform(local_point_b);

						const auto separation = (point_b - point_a).dot(normal);

						return separation;
					}

					case AxisType::FACE_A:
					{
						BPP_ASSERT(index_b != ShapeProxy::invalid_index);

						const auto local_point_a = local_witness_;
						const auto local_point_b = proxy_b.points[index_b];

						const auto point_a = transform_a.transform(local_point_a);
						const auto point_b = transform_b.transform(local_point_b);

						const auto separation = (point_b - point_a).dot(normal);

						return separation;
					}

					case AxisType::FACE_B:
					{
						BPP_ASSERT(index_a != ShapeProxy::invalid_index);

						const auto local_point_a = proxy_a.points[index_a];
						const auto local_point_b = local_witness_;

						const auto point_a = transform_a.transform(local_point_a);
						const auto point_b = transform_b.transform(local_point_b);

						const auto separation = (point_a - point_b).dot(normal);

						return separation;
					}
				}

				BPP_COMPILER_UNREACHABLE();
			}

			/// Evaluate separation with specific transforms
			[[nodiscard]] auto evaluate_with_transform(const Transform& transform_a, const Transform& transform_b) const noexcept -> float
			{
				switch (axis_type_)
				{
					case AxisType::POINTS:
					{
						const auto normal = transform_a.rotation.rotate(local_axis_);
						return evaluate_with_indices(transform_a, transform_b, normal, cached_index_a_, cached_index_b_);
					}

					case AxisType::FACE_A:
					{
						const auto normal = transform_a.rotation.rotate(flip_axis_ ? -local_axis_ : local_axis_);
						return evaluate_with_indices(transform_a, transform_b, normal, ShapeProxy::invalid_index, cached_index_b_);
					}

					case AxisType::FACE_B:
					{
						const auto normal = transform_b.rotation.rotate(flip_axis_ ? -local_axis_ : local_axis_);
						return evaluate_with_indices(transform_a, transform_b, normal, cached_index_a_, ShapeProxy::invalid_index);
					}
				}

				BPP_COMPILER_UNREACHABLE();
			}

		public:
			SeparationSolver(
				const SimplexCache& cache,
				const ShapeProxy& proxy_a,
				const ShapeProxy& proxy_b,
				const Sweep& sweep_a,
				const Sweep& sweep_b,
				const float t
			) noexcept
				: proxy_a_{proxy_a},
				  proxy_b_{proxy_b},
				  sweep_a_{sweep_a},
				  sweep_b_{sweep_b},
				  local_witness_{Vec2::zero},
				  local_axis_{Vec2::zero},
				  axis_type_{AxisType::POINTS},
				  cached_index_a_{ShapeProxy::invalid_index},
				  cached_index_b_{ShapeProxy::invalid_index},
				  flip_axis_{false}
			{
				BPP_ASSERT(cache.type == SimplexCache::Type::POINT or cache.type == SimplexCache::Type::LINE_SEGMENT);

				const auto transform_a = sweep_a_.transform_of(t);
				const auto transform_b = sweep_b_.transform_of(t);

				initialize_from_cache(cache, transform_a, transform_b);
			}

			/// Evaluate separation at time t using cached witness points
			[[nodiscard]] auto evaluate(const float t) const noexcept -> float
			{
				const auto& sweep_a = sweep_a_;
				const auto& sweep_b = sweep_b_;

				const auto transform_a = sweep_a.transform_of(t);
				const auto transform_b = sweep_b.transform_of(t);

				return evaluate_with_transform(transform_a, transform_b);
			}

			/// Find minimum separation at time t and update witness points
			[[nodiscard]] auto find_min_separation(float t) noexcept -> float
			{
				const auto& proxy_a = proxy_a_.get();
				const auto& proxy_b = proxy_b_.get();
				const auto& sweep_a = sweep_a_;
				const auto& sweep_b = sweep_b_;

				const auto transform_a = sweep_a.transform_of(t);
				const auto transform_b = sweep_b.transform_of(t);

				switch (axis_type_)
				{
					case AxisType::POINTS:
					{
						// For point-to-point axis, find support points in opposite directions
						const auto axis_world = transform_a.rotation.rotate(local_axis_);
						const auto axis_a_local = transform_a.rotation.inv_rotate(axis_world);
						const auto axis_b_local = transform_b.rotation.inv_rotate(-axis_world);

						cached_index_a_ = proxy_a.find_support(axis_a_local);
						cached_index_b_ = proxy_b.find_support(axis_b_local);

						return evaluate_with_indices(transform_a, transform_b, axis_world, cached_index_a_, cached_index_b_);
					}

					case AxisType::FACE_A:
					{
						// For face A axis, find the deepest point on shape B
						const auto normal_world = transform_a.rotation.rotate(local_axis_);
						const auto search_dir = transform_b.rotation.inv_rotate(-normal_world);

						cached_index_a_ = ShapeProxy::invalid_index;
						cached_index_b_ = proxy_b.find_support(search_dir);

						return evaluate_with_indices(transform_a, transform_b, normal_world, ShapeProxy::invalid_index, cached_index_b_);
					}

					case AxisType::FACE_B:
					{
						// For face B axis, find the deepest point on shape A
						const auto normal_world = transform_b.rotation.rotate(local_axis_);
						const auto search_dir = transform_a.rotation.inv_rotate(-normal_world);

						cached_index_a_ = proxy_a.find_support(search_dir);
						cached_index_b_ = ShapeProxy::invalid_index;

						return evaluate_with_indices(transform_a, transform_b, normal_world, cached_index_a_, ShapeProxy::invalid_index);
					}
				}

				BPP_COMPILER_UNREACHABLE();
			}

			/// Get the world normal at time t
			[[nodiscard]] auto world_normal(const float t) const noexcept -> Vec2
			{
				const auto& sweep_a = sweep_a_;
				const auto& sweep_b = sweep_b_;

				const auto transform_a = sweep_a.transform_of(t);
				const auto transform_b = sweep_b.transform_of(t);

				switch (axis_type_)
				{
					case AxisType::POINTS:
					{
						return transform_a.rotation.rotate(local_axis_);
					}

					case AxisType::FACE_A:
					{
						return transform_a.rotation.rotate(flip_axis_ ? -local_axis_ : local_axis_);
					}

					case AxisType::FACE_B:
					{
						return transform_b.rotation.rotate(flip_axis_ ? -local_axis_ : local_axis_);
					}
				}

				BPP_COMPILER_UNREACHABLE();
			}

			/// Get witness points in world coordinates at time t
			[[nodiscard]] auto witness_points(const float t) const noexcept -> std::pair<Vec2, Vec2>
			{
				const auto& proxy_a = proxy_a_.get();
				const auto& proxy_b = proxy_b_.get();
				const auto& sweep_a = sweep_a_;
				const auto& sweep_b = sweep_b_;

				const auto [local_point_a, local_point_b] = [&] noexcept -> std::pair<Vec2, Vec2>
				{
					switch (axis_type_)
					{
						case AxisType::POINTS:
						{
							const auto a = proxy_a.points[cached_index_a_ == ShapeProxy::invalid_index ? 0 : cached_index_a_];
							const auto b = proxy_b.points[cached_index_b_ == ShapeProxy::invalid_index ? 0 : cached_index_b_];

							return {a, b};
						}
						case AxisType::FACE_A:
						{
							const auto a = local_witness_;
							const auto b = proxy_b.points[cached_index_b_ == ShapeProxy::invalid_index ? 0 : cached_index_b_];

							return {a, b};
						}
						case AxisType::FACE_B:
						{
							const auto a = proxy_a.points[cached_index_a_ == ShapeProxy::invalid_index ? 0 : cached_index_a_];
							const auto b = local_witness_;

							return {a, b};
						}
					}

					BPP_COMPILER_UNREACHABLE();
				}();

				const auto transform_a = sweep_a.transform_of(t);
				const auto transform_b = sweep_b.transform_of(t);
				const auto point_a = transform_a.transform(local_point_a);
				const auto point_b = transform_b.transform(local_point_b);

				return {point_a, point_b};
			}
		};
	}

	auto TOI::compute(const TOIInput& input) noexcept -> TOI
	{
		BPP_ASSERT(input.valid());

		// Early exit for zero motion
		if (
			//
			input.sweep_a.c1 == input.sweep_a.c2 and
			input.sweep_a.q1 == input.sweep_a.q2 and
			//
			input.sweep_b.c1 == input.sweep_b.c2 and
			input.sweep_b.q1 == input.sweep_b.q2
		)
		{
			// Static case, use distance query
			const DistanceInput distance_input
			{
					.proxy_a = input.proxy_a,
					.proxy_b = input.proxy_b,
					.transform_a = input.sweep_a.transform_of(0),
					.transform_b = input.sweep_b.transform_of(0),
					.use_radii = true,
			};

			const auto distance = Distance::compute(distance_input);

			if (distance.distance <= 0)
			{
				const auto point = (distance.point_a + distance.point_b) * .5f;

				return
				{
						.state = TOIState::OVERLAPPED,
						.point = point,
						.normal = Vec2::zero,
						.fraction = 0,
						.separation = distance.distance
				};
			}

			return
			{
					.state = TOIState::SEPARATED,
					.point = Vec2::zero,
					.normal = Vec2::zero,
					.fraction = input.max_fraction,
					.separation = distance.distance
			};
		}

		const auto linear_slop = BPP_LINEAR_SLOP;
		const auto total_radius = input.proxy_a.radius + input.proxy_b.radius;
		const auto tolerance = linear_slop * .25f;
		const auto target = std::ranges::max(linear_slop, total_radius - linear_slop);

		// Main TOI algorithm with conservative advancement
		float t1 = 0;
		SimplexCache cache = SimplexCache::zero;

		for (std::size_t distance_iteration = 0; distance_iteration < BPP_COLLISION_DISTANCE_MAX_ITERATIONS; ++distance_iteration)
		{
			// Compute distance at current time
			const auto transform_a = input.sweep_a.transform_of(t1);
			const auto transform_b = input.sweep_b.transform_of(t1);

			DistanceInput distance_input
			{
					.proxy_a = input.proxy_a,
					.proxy_b = input.proxy_b,
					.transform_a = transform_a,
					.transform_b = transform_b,
					.use_radii = false,
			};

			const auto distance = Distance::compute(distance_input, cache);

			// Check for overlap
			if (distance.distance <= 0)
			{
				// Initial or intermediate overlap
				const auto point_a = multiply_add(distance.point_a, input.proxy_a.radius, distance.normal);
				const auto point_b = multiply_sub(distance.point_b, input.proxy_b.radius, distance.normal);
				const auto point = (point_a + point_b) * .5f;

				const auto start = t1 == 0; // NOLINT(clang-diagnostic-float-equal)

				return
				{
						.state = start ? TOIState::OVERLAPPED : TOIState::HIT,
						.point = point,
						.normal = start ? Vec2::zero : distance.normal,
						.fraction = t1,
						.separation = distance.distance,
				};
			}

			// Check if we've reached target separation
			if (distance.distance <= target + tolerance)
			{
				// Collision detected
				const auto point_a = multiply_add(distance.point_a, input.proxy_a.radius, distance.normal);
				const auto point_b = multiply_sub(distance.point_b, input.proxy_b.radius, distance.normal);
				const auto point = (point_a + point_b) * .5f;

				return
				{
						.state = TOIState::HIT,
						.point = point,
						.normal = distance.normal,
						.fraction = t1,
						.separation = distance.distance
				};
			}

			// Initialize separation solver with current simplex
			SeparationSolver solver{cache, input.proxy_a, input.proxy_b, input.sweep_a, input.sweep_b, t1};

			// Try to find time of impact along this separating axis
			float t2 = input.max_fraction;
			bool found_hit = false;

			for (std::size_t push_back_iteration = 0; push_back_iteration < BPP_MAX_POLYGON_VERTICES; ++push_back_iteration)
			{
				// Find minimum separation at t2
				const auto s2 = solver.find_min_separation(t2);

				// Check if shapes remain separated at end time
				if (s2 > target + tolerance)
				{
					// No collision in this interval
					t1 = t2;
					break;
				}

				// Check if we're close enough to target
				if (s2 > target - tolerance)
				{
					// Advance and continue with new axis
					t1 = t2;
					found_hit = true;
					break;
				}

				// Get separation at start time
				const auto s1 = solver.evaluate(t1);

				// Check for root bracketing failure
				if (s1 < target - tolerance)
				{
					// Root finder would fail, return best estimate
					const auto normal = solver.world_normal(t1);
					const auto [point_a, point_b] = solver.witness_points(t1);
					const auto contact_point = (point_a + point_b) * .5f;

					return {.state = TOIState::HIT, .point = contact_point, .normal = normal, .fraction = t1, .separation = s1};
				}

				// Check if already touching at t1
				if (s1 <= target + tolerance)
				{
					const auto normal = solver.world_normal(t1);
					const auto [point_a, point_b] = solver.witness_points(t1);
					const auto contact_point = (point_a + point_b) * .5f;

					return {.state = TOIState::HIT, .point = contact_point, .normal = normal, .fraction = t1, .separation = s1};
				}

				const auto find_root_bracketed = [&solver, target, tolerance](
					//
					float a,
					float fa,
					//
					float b,
					float fb
				) noexcept -> float
				{
					for (std::size_t root_iteration = 0, max_root_iterations = 50; root_iteration < max_root_iterations; ++root_iteration)
					{
						// Use a combination of secant and bisection methods
						const auto t = [&] noexcept -> float
						{
							if (root_iteration & 1)
							{
								const auto denominator = fb - fa;
								// Secant rule to improve convergence.
								return a + (target - fa) * (b - a) / denominator;
							}

							// Bisection to guarantee progress.
							return (a + b) * .5f;
						}();

						const auto ft = solver.evaluate(t) - target;

						// Check convergence
						if (std::abs(ft) < tolerance)
						{
							return t;
						}

						// Update bracket
						if (ft > 0)
						{
							a = t;
							fa = ft;
						}
						else
						{
							b = t;
							fb = ft;
						}

						// Check bracket size
						if (std::abs(b - a) < tolerance)
						{
							break;
						}
					}

					// Return best estimate
					return (a + b) * .5f;
				};

				auto root = find_root_bracketed(t1, s1, t2, s2);
				if (root < t1 or root > t2)
				{
					// Root finder failed, use conservative estimate
					root = (t1 + t2) * .5f;
				}

				t2 = root;
			}

			if (not found_hit and t2 >= input.max_fraction)
			{
				// No collision found in entire interval
				return
				{
						.state = TOIState::SEPARATED,
						.point = Vec2::zero,
						.normal = Vec2::zero,
						.fraction = input.max_fraction,
						.separation = solver.evaluate(input.max_fraction),
				};
			}

			if (t1 >= input.max_fraction)
			{
				// Exceeded time limit
				return
				{
						.state = TOIState::SEPARATED,
						.point = Vec2::zero,
						.normal = Vec2::zero,
						.fraction = input.max_fraction,
						.separation = solver.evaluate(input.max_fraction),
				};
			}
		}

		// Algorithm failed to converge
		return
		{
				.state = TOIState::FAILED,
				.point = Vec2::zero,
				.normal = Vec2::zero,
				.fraction = t1,
				.separation = 0,
		};
	}
}
