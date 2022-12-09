#include "Feature.h"


Feature::Feature() :
	type(FT_NONE)
{};


Feature::Feature(const Feature &rhs) :
	segmentP1(rhs.segmentP1),
	segmentP2(rhs.segmentP2),
	type(rhs.type)
{};


Feature::Feature(const Vector2& _point) :
	point(_point),
	type(FT_POINT)
{};


Feature::Feature(const Vector2& _segmentP1, const Vector2& _segmentP2) :
	segmentP1(_segmentP1),
	segmentP2(_segmentP2),
	type(FT_SEGMENT)
{};


Feature& Feature::operator= (const Feature& rhs)
{
	segmentP1 = rhs.segmentP1;
	segmentP2 = rhs.segmentP2;
	type = rhs.type;
	return *this;
};


Feature& Feature::operator+= (const Vector2& _v)
{
	segmentP1 += _v;
	segmentP2 += _v;
	return *this;
};


Feature Feature::operator+ (const Vector2& _v)
{
	return Feature(*this) += _v;
};


void Feature::set(const Vector2 &_point)
{
	point = _point;
	type = FT_POINT;
};


void Feature::set(const Vector2& _segmentP1, const Vector2& _segmentP2)
{
	segmentP1 = _segmentP1;
	segmentP2 = _segmentP2;
	type = FT_SEGMENT;
};


Feature operator* (const Quaternion& _q, const Feature& _f)
{
	if (_f.type == Feature::FT_SEGMENT)
		return Feature(_q * _f.segmentP1, _q * _f.segmentP2);
	return Feature(_q * _f.point);
};
