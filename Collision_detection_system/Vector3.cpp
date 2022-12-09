#include "Vector3.h"



Vector3::Vector3(float _s) :
	x(_s),
	y(_s),
	z(_s)
{};


Vector3::Vector3(float _x, float _y, float _z) :
	x(_x),
	y(_y),
	z(_z)
{};


Vector3::Vector3(const Vector3& _v) :
	x(_v.x),
	y(_v.y),
	z(_v.z)
{};


Vector3& Vector3::operator= (const Vector3& _v)
{
	x = _v.x;
	y = _v.y;
	z = _v.z;
	return *this;
};


Vector3& Vector3::operator+= (const Vector3& _v)
{
	x += _v.x;
	y += _v.y;
	z += _v.z;
	return *this;
};


Vector3& Vector3::operator-= (const Vector3& _v)
{
	x -= _v.x;
	y -= _v.y;
	z -= _v.z;
	return *this;
};


Vector3& Vector3::operator+= (float _s)
{
	x += _s;
	y += _s;
	z += _s;
	return *this;
};


Vector3& Vector3::operator-= (float _s)
{
	x -= _s;
	y -= _s;
	z -= _s;
	return *this;
};


Vector3& Vector3::operator*= (float _s)
{
	x *= _s;
	y *= _s;
	z *= _s;
	return *this;
};


Vector3& Vector3::operator/= (float _s)
{
	return *this *= 1.0f / _s;
};


Vector3 Vector3::operator+ (const Vector3& _v) const
{
	return Vector3(*this) += _v;
};


Vector3 Vector3::operator- (const Vector3& _v) const
{
	return Vector3(*this) -= _v;
};


Vector3 Vector3::operator+ (float _s) const
{
	return Vector3(*this) += _s;
};


Vector3 Vector3::operator- (float _s) const
{
	return Vector3(*this) -= _s;
};


Vector3 Vector3::operator* (float _s) const
{
	return Vector3(*this) *= _s;
};


Vector3 Vector3::operator/ (float _s) const
{
	return Vector3(*this) /= _s;
};


Vector3 Vector3::operator- () const
{
	return Vector3(-this->x, -this->y, -this->z);
};


float Vector3::length() const
{
	return std::sqrtf(x * x + y * y + z * z);
};


void Vector3::normalize()
{
	float lenSq = x * x + y * y + z * z;
	if (std::abs(lenSq - 1.0f) > EPSILON)
	{
		*this /= std::sqrtf(lenSq);
	}
};
