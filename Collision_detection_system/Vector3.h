#pragma once

#include <cmath>
#include "Constants.h"



struct Vector3
{
	union
	{
		float v[3];
		struct
		{
			float x;
			float y;
			float z;
		};
	};

	Vector3(float _s = 0.0f);
	Vector3(float _x, float _y, float _z);
	Vector3(const Vector3& _v);

	Vector3& operator= (const Vector3& _v);
	Vector3& operator+= (const Vector3& _v);
	Vector3& operator-= (const Vector3& _v);

	Vector3& operator+= (float _s);
	Vector3& operator-= (float _s);
	Vector3& operator*= (float _s);
	Vector3& operator/= (float _s);

	Vector3 operator+ (const Vector3& _v) const;
	Vector3 operator- (const Vector3& _v) const;

	Vector3 operator+ (float _s) const;
	Vector3 operator- (float _s) const;
	Vector3 operator* (float _s) const;
	Vector3 operator/ (float _s) const;
	Vector3 operator- () const;

	float length() const;

	void normalize();

};
