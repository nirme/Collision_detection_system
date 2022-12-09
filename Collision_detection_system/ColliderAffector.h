#pragma once

#include <memory>
#include <string>
#include <vector>
#include "ImpulseStructs.h"


class Collider;

class ColliderAffector
{
private:
	char name[AFFECTOR_ID_LEN];
	bool staticEffect;

public:
	ColliderAffector(const char _name[AFFECTOR_ID_LEN], bool _isStatic);
	virtual ~ColliderAffector();

	const char* getName() const;
	bool isStatic() const;

	virtual Force generateForce(const Collider& c) const = 0;
};

typedef std::shared_ptr<ColliderAffector> ColliderAffectorSPtr;



class StaticAccelerationAffector : public ColliderAffector
{
private:
	Acceleration acceleration;

public:
	StaticAccelerationAffector(const char _name[AFFECTOR_ID_LEN], const Acceleration& _acceleration = { {0.0f, 0.0f}, 0.0f });
	virtual ~StaticAccelerationAffector();

	void setAcceleration(const Acceleration& _acceleration);
	
	virtual Force generateForce(const Collider& c) const;
};


class LinearForceAffector : public ColliderAffector
{
private:
	Force force;
	Vector2 position;

public:
	LinearForceAffector(const char _name[AFFECTOR_ID_LEN], const Force& _force = { {0.0f, 0.0f}, 0.0f }, Vector2 _position = { 0.0f, 0.0f });
	virtual ~LinearForceAffector();

	const Force& getForce() const;
	void setForce(const Force& _force);

	virtual Force generateForce(const Collider& c) const;
};