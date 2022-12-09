#include "Vector2.h"


Vector2 UNIT_X = Vector2(1.0f, 0.0f);
Vector2 UNIT_Y = Vector2(0.0f, 1.0f);


Vector2::Vector2(float _s) :
	x(_s),
	y(_s)
{};


Vector2::Vector2(float _x, float _y) :
	x(_x),
	y(_y)
{};


Vector2::Vector2(const Vector2& _v) :
	x(_v.x),
	y(_v.y)
{};


Vector2& Vector2::operator= (const Vector2& _v)
{
	x = _v.x;
	y = _v.y;
	return *this;
};


Vector2& Vector2::operator+= (const Vector2& _v)
{
	x += _v.x;
	y += _v.y;
	return *this;
};


Vector2& Vector2::operator-= (const Vector2& _v)
{
	x -= _v.x;
	y -= _v.y;
	return *this;
};


Vector2& Vector2::operator+= (float _s)
{
	x += _s;
	y += _s;
	return *this;
};


Vector2& Vector2::operator-= (float _s)
{
	return *this += -_s;
};


Vector2& Vector2::operator*= (float _s)
{
	x *= _s;
	y *= _s;
	return *this;
};


Vector2& Vector2::operator/= (float _s)
{
	return *this *= 1.0f / _s;
};


Vector2 Vector2::operator+ (const Vector2& _v) const
{
	return Vector2(*this) += _v;
};


Vector2 Vector2::operator- (const Vector2& _v) const
{
	return Vector2(*this) -= _v;
};


Vector2 Vector2::operator+ (float _s) const
{
	return Vector2(*this) += _s;
};


Vector2 Vector2::operator- (float _s) const
{
	return Vector2(*this) -= _s;
};


Vector2 Vector2::operator* (float _s) const
{
	return Vector2(*this) *= _s;
};


Vector2 Vector2::operator/ (float _s) const
{
	return Vector2(*this) /= _s;
};


Vector2 Vector2::operator- () const
{
	return Vector2(-this->x, -this->y);
};


float Vector2::length() const
{
	return std::sqrt(x * x + y * y);
};


float Vector2::lengthSq() const
{
	return x * x + y * y;
};


Vector2 Vector2::normalize() const
{
	float len = 1.0f / length();
	return *this * len;
};


bool Vector2::isNormalized() const
{
	return std::abs(1.0f - x * x - y * y) < EPSILON;
};


bool Vector2::isZero() const
{
	return std::abs(x) < EPSILON && std::abs(y) < EPSILON;
};


bool Vector2::operator== (const Vector2& _v) const
{
	return (std::abs(x - _v.x) < EPSILON && std::abs(y - _v.y) < EPSILON);
};


bool Vector2::operator!= (const Vector2& _v) const
{
	return !(*this == _v);
};


Vector2 operator* (float lhs, const Vector2 &rhs)
{
	return rhs * lhs;
};
