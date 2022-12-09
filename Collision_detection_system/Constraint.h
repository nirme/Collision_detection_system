#pragma once

#include "Constants.h"
#include "Vector2.h"
#include "ImpulseStructs.h"
#include "Collider.h"


struct ConstraintValue
{
	Jacobian J1, J2;
	float C;
};

class Constraint
{
protected:
	char name[CONSTRAINT_ID_LEN];
	float beta;
	float lambdaMin, lambdaMax;

public:
	Constraint(const char _name[CONSTRAINT_ID_LEN], float _lambdaMin = N_INF, float _lambdaMax = P_INF);
	virtual ~Constraint();

	void setName(const char _name[CONSTRAINT_ID_LEN]);
	const char* getName() const;

	virtual ConstraintValue calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const = 0;

	void setCoefficient(float _beta);
	float getCoefficient();

	void setLambdaLimits(float _lambdaMin, float _lambdaMax);
	float getLambdaMin() const;
	float getLambdaMax() const;
	float clampLambda(float lambda) const;
};

typedef std::unique_ptr<Constraint> ConstraintUPtr;
typedef std::shared_ptr<Constraint> ConstraintSPtr;


class DistanceConstraint : public Constraint
{
protected:
	Vector2 r1;
	Vector2 r2;
	float L;

public:
	DistanceConstraint(const char _name[CONSTRAINT_ID_LEN], const Vector2& _r1, const Vector2& _r2, float _L);
	virtual ~DistanceConstraint();

	ConstraintValue calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const;
};


class NormalConstraint : public Constraint
{
protected:
	Vector2 n; //collision normal
	Vector2 r1;
	Vector2 r2;
	float e; //restitution coefficient

public:
	NormalConstraint(const char _name[CONSTRAINT_ID_LEN], const Vector2& _n, const Vector2& _r1, const Vector2& _r2, float _e);
	virtual ~NormalConstraint();

	ConstraintValue calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const;
};


class FrictionConstraint : public Constraint
{
protected:
	Vector2 t; //collision tangent, in 2d 
	Vector2 r1;
	Vector2 r2;

public:
	FrictionConstraint(const char _name[CONSTRAINT_ID_LEN]);
	virtual ~FrictionConstraint();

	ConstraintValue calculate(float timedelta, const Velocity& v1, const Position& p1, const Velocity& v2, const Position& p2) const;
};

