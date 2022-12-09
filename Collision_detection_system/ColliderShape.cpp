#include "ColliderShape.h"
#include "Collider.h"


const Vector2& ColliderShape::getParentPosition() const
{
	return parent->getOriginPosition();
};


const Quaternion& ColliderShape::getParentRotation() const
{
	return parent->getPosition().angular;
};


void ColliderShape::setMMOI(float _mmoi)
{
	mmoi = _mmoi;
};


void ColliderShape::setCenterOfMass(Vector2 c)
{
	centerOfMass = c;
};


ColliderShape::ColliderShape() :
	parent(nullptr),
	mmoi(0.0f),
	mass(0.0f),
	centerOfMass(0.0f)
{};


ColliderShape::~ColliderShape()
{};


void ColliderShape::setParent(Collider *_parent)
{
	parent = _parent;
};


void ColliderShape::setMass(float _mass)
{
	mass = _mass;
	updateCenterAndMMOI();
};


float ColliderShape::getMass() const
{
	return mass;
};


Vector2 ColliderShape::getCenterOfMass() const // local space
{
	return centerOfMass;
};


float ColliderShape::getMMOI() const
{
	return mmoi;
};


void ColliderShape::setMaterialId(MaterialID id)
{
	materialId = id;
};


MaterialID ColliderShape::getMaterialId() const
{
	return materialId;
};


Vector2 ColliderShape::getPosition(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	return wPosition + (wRotation * getLocalPosition());
};

