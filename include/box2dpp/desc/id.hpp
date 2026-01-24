// T// This file is part of box2dpp
// Copyright (C) 2025 Life4gal <life4gal@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#pragma once

#include <bit>

#include <box2dpp/version.hpp>

namespace box2dpp
{
	// todo: The content of the ID depends on the design of the World, or more precisely, on who manages the world—whether it's us or the users.

	namespace id_detail
	{
		template<std::uint64_t Size, std::uint64_t Alignment>
		class Opaque
		{
		public:
			alignas(Alignment) unsigned char data[Size]{};
		};
	}

	/// World id references a world instance.
	class WorldId final : public id_detail::Opaque<
				// index
				sizeof(std::uint16_t) +
				// generation
				sizeof(std::uint16_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	/// Body id references a body instance.
	class BodyId final : public id_detail::Opaque<
				// index
				sizeof(std::int32_t) +
				// world
				sizeof(std::uint16_t) +
				// generation
				sizeof(std::uint16_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	/// Shape id references a shape instance.
	class ShapeId final : public id_detail::Opaque<
				// index
				sizeof(std::int32_t) +
				// world
				sizeof(std::uint16_t) +
				// generation
				sizeof(std::uint16_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	/// Chain id references a chain instances.
	class ChainId final : public id_detail::Opaque<
				// index
				sizeof(std::int32_t) +
				// world
				sizeof(std::uint16_t) +
				// generation
				sizeof(std::uint16_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	/// Joint id references a joint instance.
	class JointId final : public id_detail::Opaque<
				// index
				sizeof(std::int32_t) +
				// world
				sizeof(std::uint16_t) +
				// generation
				sizeof(std::uint16_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	/// Contact id references a contact instance.
	class ContactId final : public id_detail::Opaque<
				// index
				sizeof(std::int32_t) +
				// world
				sizeof(std::uint32_t) +
				// generation
				sizeof(std::uint32_t),
				// alignment
				alignof(std::uint32_t)
			> {};

	namespace id_detail
	{
		class alignas(box2dpp::WorldId) WorldId final
		{
		public:
			std::uint16_t index1;
			std::uint16_t generation;
		};

		class alignas(box2dpp::BodyId) BodyId final
		{
		public:
			std::int32_t index1;
			std::uint16_t world0;
			std::uint16_t generation;
		};

		class alignas(box2dpp::ShapeId) ShapeId final
		{
		public:
			std::int32_t index1;
			std::uint16_t world0;
			std::uint16_t generation;
		};

		class alignas(box2dpp::ChainId) ChainId final
		{
		public:
			std::int32_t index1;
			std::uint16_t world0;
			std::uint16_t generation;
		};

		class alignas(box2dpp::JointId) JointId final
		{
		public:
			std::int32_t index1;
			std::uint16_t world0;
			std::uint16_t generation;
		};

		class alignas(box2dpp::ContactId) ContactId final
		{
		public:
			std::int32_t index1;
			// std::uint16_t
			std::uint32_t world0;
			std::uint32_t generation;
		};
	}

	template<typename T>
	[[nodiscard]] constexpr auto underlying_of(const T& id) noexcept -> auto
	{
		if constexpr (std::is_same_v<T, WorldId>)
		{
			return std::bit_cast<id_detail::WorldId>(id);
		}
		else if constexpr (std::is_same_v<T, BodyId>)
		{
			return std::bit_cast<id_detail::BodyId>(id);
		}
		else if constexpr (std::is_same_v<T, ShapeId>)
		{
			return std::bit_cast<id_detail::ShapeId>(id);
		}
		else if constexpr (std::is_same_v<T, ChainId>)
		{
			return std::bit_cast<id_detail::ChainId>(id);
		}
		else if constexpr (std::is_same_v<T, JointId>)
		{
			return std::bit_cast<id_detail::JointId>(id);
		}
		else if constexpr (std::is_same_v<T, ContactId>)
		{
			return std::bit_cast<id_detail::ContactId>(id);
		}
		else
		{
			BPP_COMPILER_UNREACHABLE();
		}
	}
}
