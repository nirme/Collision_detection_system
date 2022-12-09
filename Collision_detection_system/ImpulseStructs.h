#pragma once

#include "Constants.h"
#include <memory>
#include "Vector2.h"
#include "Quaternion.h"
#include "Vector3.h"



struct Position
{
	union {
		float data[4];
		struct {
			Vector2 linear;
			Quaternion angular;
		};
	};

	Position(const Vector2& _linear = Vector2(0.0f, 0.0f), const Quaternion& _angular = Quaternion(0.0f, 0.0f));
	Position(const Position& rhs);

	Position& operator= (const Position& rhs);
};


struct MassMatrix
{
	float m;
	float I;
};


struct Velocity
{
	union
	{
		float data[3];
		struct {
			Vector2 linear;
			float angular;
		};
	};

	// define ctors and assignment as union deletes those
	Velocity();
	Velocity(const Velocity& rhs);
	Velocity(Vector2 l, float a);

	Velocity& operator= (const Velocity& rhs);
	Velocity& operator+= (const Velocity& rhs);
};

typedef Velocity Jacobian;
typedef Velocity Acceleration;
typedef Velocity Force;
typedef Velocity Impulse;



Velocity operator+ (const Velocity& v1, const Velocity& v2);

float operator* (const Jacobian &j, const Velocity &v);
Force operator* (const Jacobian &j, float lambda);


struct IndexedJacobian
{
	int32_t b1, b2;
	Jacobian J1, J2;
};


Velocity operator* (const MassMatrix& mm, const Velocity& v);
Velocity operator* (const Velocity& v, const MassMatrix& mm);
