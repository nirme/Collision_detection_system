#pragma once

#include <cmath>
#include "Constants.h"


struct Vector2
{
	union
	{
		float v[2];
		struct
		{
			float x;
			float y;
		};
	};


	explicit Vector2(float _s = 0.0f);
	Vector2(float _x, float _y);
	Vector2(const Vector2& _v);

	Vector2& operator= (const Vector2& _v);
	Vector2& operator+= (const Vector2& _v);
	Vector2& operator-= (const Vector2& _v);

	Vector2& operator+= (float _s);
	Vector2& operator-= (float _s);
	Vector2& operator*= (float _s);
	Vector2& operator/= (float _s);

	Vector2 operator+ (const Vector2& _v) const;
	Vector2 operator- (const Vector2& _v) const;

	Vector2 operator+ (float _s) const;
	Vector2 operator- (float _s) const;
	Vector2 operator* (float _s) const;
	Vector2 operator/ (float _s) const;
	Vector2 operator- () const;

	float length() const;
	float lengthSq() const;
	Vector2 normalize() const;
	bool isNormalized() const;
	bool isZero() const;

	bool operator== (const Vector2& _v) const;
	bool operator!= (const Vector2& _v) const;


	static const Vector2 UNIT_X;
	static const Vector2 UNIT_Y;
};

Vector2 operator* (float lhs, const Vector2 &rhs);
