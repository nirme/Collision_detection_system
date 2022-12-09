#include "ImpulseStructs.h"



Position::Position(const Vector2& _linear, const Quaternion& _angular) :
	linear(_linear), 
	angular(_angular)
{};


Position::Position(const Position& rhs) : 
	linear(rhs.linear), 
	angular(rhs.angular)
{};


Position& Position::operator= (const Position& rhs)
{
	linear = rhs.linear;
	angular = rhs.angular;
	return *this;
};


Velocity::Velocity() :
	linear(0.0f, 0.0f),
	angular(0.0f)
{};


Velocity::Velocity(const Velocity& rhs) :
	linear(rhs.linear), 
	angular(rhs.angular)
{};


Velocity::Velocity(Vector2 l, float a) :
	linear(l), 
	angular(a)
{};


Velocity& Velocity::operator= (const Velocity& rhs)
{
	linear = rhs.linear;
	angular = rhs.angular;
	return *this;
};


Velocity& Velocity::operator+= (const Velocity& rhs)
{
	linear += rhs.linear;
	angular += rhs.angular;
	return *this;
};


Velocity operator+ (const Velocity& v1, const Velocity& v2)
{
	return Velocity(v1.linear + v2.linear, v1.angular + v2.angular);
};


float operator* (const Jacobian &j, const Velocity &v)
{
	// rewrite to SIMD
	return j.data[0] * v.data[0] + j.data[1] * v.data[1] + j.data[2] * v.data[2];
};


Force operator* (const Jacobian &j, float lambda)
{
	return Force(j.linear * lambda, j.angular * lambda);
};


Velocity operator* (const MassMatrix& mm, const Velocity& v)
{
	return { {mm.m * v.linear.x, mm.m * v.linear.y}, mm.I * v.angular };
};

Velocity operator* (const Velocity& v, const MassMatrix& mm)
{
	return mm * v;
};
