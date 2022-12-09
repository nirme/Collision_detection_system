#pragma once
#include "Vector2.h"
#include "Quaternion.h"
#include "Math2D.h"


struct Feature
{
	union {
		struct {
			Vector2 segmentP1;
			Vector2 segmentP2;
		};
		Vector2 point;
	};

	enum FEATURE_TYPE : unsigned short {
		FT_POINT = 0,
		FT_SEGMENT = 1,
		FT_NONE = 0xFF,
	} type;

	Feature();
	Feature(const Feature &rhs);
	Feature(const Vector2& _point);
	Feature(const Vector2& _segmentP1, const Vector2& _segmentP2);

	Feature& operator= (const Feature& rhs);
	Feature& operator+= (const Vector2& _v);
	Feature operator+ (const Vector2& _v);

	void set(const Vector2 &_point);
	void set(const Vector2& _segmentP1, const Vector2& _segmentP2);
};

Feature operator* (const Quaternion& _q, const Feature& _f);
