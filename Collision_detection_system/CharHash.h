#pragma once

#include <cstdint>
#include <functional>


template <typename ResultT, ResultT OffsetBasis, ResultT Prime>
struct FNV1a_hash
{
	static_assert(std::is_unsigned<ResultT>::value, "need unsigned integer");

	constexpr ResultT digest(const void *const data, const std::size_t size) noexcept
	{
		auto cdata = static_cast<const unsigned char *const>(data);
		ResultT hash = OffsetBasis;

		for (auto i = 0; i < size; ++i)
			hash = (hash ^ cdata[i]) * Prime;

		return hash;
	};
};


using FNV1a_hash_32 = FNV1a_hash<std::uint32_t,
	UINT32_C(2166136261),
	UINT32_C(16777619)>;

using FNV1a_hash_64 = FNV1a_hash<std::uint64_t,
	UINT64_C(14695981039346656037),
	UINT64_C(1099511628211)>;


template <std::size_t Bits>
struct FNV1a;

template <>
struct FNV1a<32>
{
	using type = FNV1a_hash_32;
};

template <>
struct FNV1a<64>
{
	using type = FNV1a_hash_64;
};

template <std::size_t Bits>
using FNV1a_t = typename FNV1a<Bits>::type;


template <std::size_t maxLength>
struct CharHash
{
public:
	CharHash() {};

	constexpr std::size_t operator()(const char *const s) const
	{
		auto hashfn = FNV1a_t<CHAR_BIT * sizeof(std::size_t)>{};
		return hashfn.digest(s, strnlen_s(s, maxLength));
	};
};


template <std::size_t maxLength>
struct CharComparer
{
	constexpr bool operator()(const char *const lhs, const char *const rhs) const
	{
		return strncmp(lhs, rhs, maxLength) == 0;
	};
};
