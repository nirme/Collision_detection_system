#include "Quaternion.h"


Quaternion::Quaternion() :
	w(1.0f),
	z(0.0f)
{};


Quaternion::Quaternion(float _t) :
	w(std::cos(_t * 0.5f)),
	z(std::sin(_t * 0.5f))
{};


Quaternion::Quaternion(const Quaternion& _q) :
	w(_q.w),
	z(_q.z)
{};


Quaternion& Quaternion::operator*= (const Quaternion& _q)
{
	//  w = (w1w2 - z1z2)
	//  z = (w1z2 + z1w2)
	float wtmp = w * _q.w - z * _q.z;
	z = w * _q.z + z * _q.w;
	w = wtmp;
	return *this;
};


Quaternion::Quaternion(float _w, float _z) :
	w(_w), 
	z(_z)
{};


Quaternion& Quaternion::operator*= (float _s)
{
	w = w * _s;
	z = z * _s;
	return *this;
};


Quaternion Quaternion::operator- () const
{
	Quaternion t(*this);
	t.w = -t.w;
	return t;
};


Quaternion Quaternion::operator* (const Quaternion& _q) const
{
	Quaternion tmp(*this);
	return tmp *= _q;
};


Quaternion Quaternion::operator* (float _s) const
{
	Quaternion out(*this);
	out *= _s;
	return out;
};


Quaternion& Quaternion::rotate(float _t)
{
	return *this *= Quaternion(_t);
};


Quaternion::operator float() const
{
	return 2.0f * std::acos(w);
};


void Quaternion::normalize()
{
	float len = w * w + z * z;
	//if (std::abs(len - 1.0f) > EPSILON)
	{
		len = 1.0f / std::sqrt(len);
		w *= len;
		z *= len;
	}
};
