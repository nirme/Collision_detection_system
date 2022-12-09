#pragma once

#include <cstring>


struct Matrix3
{
	union
	{
		float m[9];

		struct
		{
			float m11, m12, m13;
			float m21, m22, m23;
			float m31, m32, m33;
		};
	};


	Matrix3();
	Matrix3(float _m11, float _m12, float _m13, float _m21, float _m22, float _m23,float _m31, float _m32, float _m33);
	Matrix3(const Matrix3& _m);

	Matrix3& operator= (const Matrix3& _m);
	Matrix3 operator* (const Matrix3& _m) const;

	inline Matrix3& operator*= (const Matrix3& _m)
	{
		return *this = *this * _m;
	};

	static const Matrix3 IDENTITY;
};

