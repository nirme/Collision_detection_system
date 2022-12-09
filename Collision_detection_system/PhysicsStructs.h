#pragma once

#include <functional>
#include "ColliderShape.h"


#define FEATURE_HASH(f0,f1) (((unsigned int)f0 << 16) | f1)
typedef unsigned int FeatureHash;


enum AFFECTOR_SCOPE
{
	AS_ALL = 0,
	AS_LIST = 1,
	AS_ONE = 2
};


struct MaterialPair
{
	MaterialID id1;
	MaterialID id2;

	MaterialPair() = default;
	MaterialPair(MaterialID _id1, MaterialID _id2);
};


bool operator==(const MaterialPair& lhs, const MaterialPair& rhs);

template<>
struct std::hash<MaterialPair>
{
	std::size_t operator()(MaterialPair const& mp) const noexcept
	{
		unsigned short h = mp.id1 > mp.id2 ? mp.id1 : mp.id2;
		unsigned short l = mp.id1 > mp.id2 ? mp.id2 : mp.id1;
		return (unsigned int)h << (sizeof(unsigned short) * 8) | l;
	}
};


bool operator==(const std::pair<Collider*, Collider*> &lhs, const std::pair<Collider*, Collider*> &rhs);

template<>
struct std::hash<std::pair<Collider*, Collider*>>
{
	std::size_t operator()(std::pair<Collider*, Collider*> const& p) const noexcept
	{
		return (std::size_t) p.first ^ (std::size_t) p.second;
	}
};


struct DirectionVector
{
	Vector2 direction;
	float magnitude;

	DirectionVector() = default;
	DirectionVector(Vector2 vector);
	DirectionVector(Vector2 _direction, float _magnitude);
	DirectionVector operator-() const;
	operator Vector2 () const;
};


struct Features
{
	Feature f1;
	Feature f2;

	Features() = default;
	Features(const Features& rhs);
	Features(const Feature& _f1, const Feature& _f2);

	unsigned int hash() const;

	static constexpr unsigned int HASH_POINT_POINT = FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT);
	static constexpr unsigned int HASH_POINT_SEGMENT = FEATURE_HASH(Feature::FT_POINT, Feature::FT_SEGMENT);
	static constexpr unsigned int HASH_SEGMENT_POINT = FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_POINT);
	static constexpr unsigned int HASH_SEGMENT_SEGMENT = FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_SEGMENT);
};

