#include "ColliderAffector.h"
#include "Collider.h"


ColliderAffector::ColliderAffector(const char _name[AFFECTOR_ID_LEN], bool _isStatic) :
	staticEffect(_isStatic)
{
	strcpy_s(name, AFFECTOR_ID_LEN, _name);
};


ColliderAffector::~ColliderAffector()
{};


const char* ColliderAffector::getName() const
{
	return name;
};


bool ColliderAffector::isStatic() const
{
	return staticEffect;
};



StaticAccelerationAffector::StaticAccelerationAffector(const char _name[AFFECTOR_ID_LEN], const Acceleration& _acceleration) :
	ColliderAffector(_name, true),
	acceleration(_acceleration)
{};


StaticAccelerationAffector::~StaticAccelerationAffector()
{};


void StaticAccelerationAffector::setAcceleration(const Acceleration& _acceleration)
{
	acceleration = _acceleration;
};


Force StaticAccelerationAffector::generateForce(const Collider& c) const
{
	return acceleration * c.getMassMx();
};


LinearForceAffector::LinearForceAffector(const char _name[AFFECTOR_ID_LEN], const Force& _force, Vector2 _position) :
	ColliderAffector(_name, false),
	force(_force),
	position(_position)
{
	assert(_force.angular == 0.0f && "linear force contain angular component");
};


LinearForceAffector::~LinearForceAffector()
{};


const Force& LinearForceAffector::getForce() const
{
	return force;
};


void LinearForceAffector::setForce(const Force& _force)
{
	assert(_force.angular == 0.0f && "linear force contain angular component");
	force = _force;
};


Force LinearForceAffector::generateForce(const Collider& c) const
{
	Vector2 r = c.getPosition().angular * position;
	return {force.linear, crossProduct(r, force.linear)};
};
