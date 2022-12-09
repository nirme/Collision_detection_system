#include "Constraint.h"


Constraint::Constraint(const char _name[CONSTRAINT_ID_LEN], float _lambdaMin, float _lambdaMax) :
	beta(0.0f), 
	lambdaMin(_lambdaMin),
	lambdaMax(_lambdaMax)
{
	setName(_name);
};


Constraint::~Constraint()
{};


void Constraint::setName(const char _name[CONSTRAINT_ID_LEN])
{
	strcpy_s(name, CONSTRAINT_ID_LEN, _name);
};


const char* Constraint::getName() const
{
	return name;
};


void Constraint::setCoefficient(float _beta)
{
	beta = _beta;
};


float Constraint::getCoefficient()
{
	return beta;
};


void Constraint::setLambdaLimits(float _lambdaMin, float _lambdaMax)
{
	lambdaMin = _lambdaMin;
	lambdaMax = _lambdaMax;
};


float Constraint::getLambdaMin() const
{
	return lambdaMin;
};


float Constraint::getLambdaMax() const
{
	return lambdaMax;
};


float Constraint::clampLambda(float lambda) const
{
	return std::clamp(lambda, lambdaMin, lambdaMax);
};


DistanceConstraint::DistanceConstraint(const char _name[CONSTRAINT_ID_LEN], const Vector2& _r1, const Vector2& _r2, float _L) :
	Constraint(_name),
	r1(_r1),
	r2(_r2),
	L(_L)
{};


DistanceConstraint::~DistanceConstraint()
{};


ConstraintValue DistanceConstraint::calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const
{
	Vector2 _r2 = p2.angular * r2;
	Vector2 _r1 = p1.angular * r1;
	Vector2 d = (p2.linear + _r2) - (p1.linear + _r1);

	return {
		{ -d, -(crossProduct(_r1, d)) },
		{ d, crossProduct(_r2, d) },
		0.5f * (d.lengthSq() - L * L)
		//d.length() - L
	};
};


NormalConstraint::NormalConstraint(const char _name[CONSTRAINT_ID_LEN], const Vector2& _n, const Vector2& _r1, const Vector2& _r2, float _e) : 
	Constraint(_name, 0.0f, P_INF), 
	n(_n), 
	r1(_r1), 
	r2(_r2), 
	e(_e)
{};


NormalConstraint::~NormalConstraint()
{};


ConstraintValue NormalConstraint::calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const
{
	Vector2 _r2 = p2.angular * r2;
	Vector2 _r1 = p1.angular * r1;
	float d = dotProduct(p1.linear + _r1 - p2.linear - _r2, n);

	ConstraintValue cv;
	cv.J1 = { -n, -(crossProduct(_r1, n)) };
	cv.J2 = { n, crossProduct(_r2, n) };
	cv.C = (d / timedelta * beta) - ((cv.J1 * v1 + cv.J2 * v2) * e);

	return cv;
};



FrictionConstraint::FrictionConstraint(const char _name[CONSTRAINT_ID_LEN]) :
	Constraint(_name, 0.0f, P_INF)
{};


FrictionConstraint::~FrictionConstraint()
{};


ConstraintValue FrictionConstraint::calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const
{
	Vector2 _r1 = p1.angular * r1;
	Vector2 _r2 = p2.angular * r2;
	return {
		{-t, -crossProduct(_r1, t)},
		{t, crossProduct(_r2, t)},
		0.0f };
};
