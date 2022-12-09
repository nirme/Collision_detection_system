#pragma once

#include "Constants.h"
#include <cmath>


class Quaternion
{
public:
	union
	{
		float q[2];
		struct
		{
			float w;
			float z;
		};
	};

	Quaternion();
	explicit Quaternion(float _t);
	Quaternion(float _w, float _z);
	Quaternion(const Quaternion& _q);

	/*
	quaternion * vector is a vector rotationally offset by the quaternion
		quaternion * quaternion is a quaternion with both rotations combined, therefore:
		rotation1 * rotation2 is a quaternion with both rotations, rotation1 is the orientation of the parent node and orientation2 is the orientation of the child node
	*/
	Quaternion& operator*= (const Quaternion& _q);
	Quaternion& operator*= (float _s);
	Quaternion operator- () const;
	Quaternion operator* (const Quaternion& _q) const;
	Quaternion operator* (float _s) const;

	Quaternion& rotate(float _t);

	explicit operator float() const;

	void normalize();
};
