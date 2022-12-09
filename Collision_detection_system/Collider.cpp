#include "Collider.h"


void Collider::invalidateAABB()
{
	AABBNeedUpdate = true;
};


void Collider::invalidateOriginPositon()
{
	originPositonNeedUpdate = true;
};


void Collider::pushPositionData(float timestep)
{
	PositionData pd = { timestep, position, velocity };
	positionHistory.push(pd);
};


float Collider::rollbackPositionData()
{
	PositionData pd = positionHistory.pop();

	position = pd.position;
	velocity = pd.velocity;

	// invalidate temporaries
	invalidateAABB();
	invalidateOriginPositon();

	return pd.timestep;
};


void Collider::clearPositionHistory()
{
	positionHistory.clear();
};


Collider::Collider() : 
	name("\0"), 
	broadphaseId(-1)
{
	invalidateAABB();
	invalidateOriginPositon();
};


Collider::~Collider()
{};


void Collider::setName(const char _name[COLLIDER_ID_LEN])
{
	strcpy_s(name, COLLIDER_ID_LEN, _name);
};


const char* Collider::getName() const
{
	return name;
};


void Collider::setPhysicsBroadPhaseId(int32_t _broadphaseId)
{
	broadphaseId = _broadphaseId;
};


int32_t Collider::getPhysicsBroadPhaseId() const
{
	return broadphaseId;
};


void Collider::setPhysicsNarrowPhaseId(int32_t _narrowphaseId)
{
	narrowphaseId = _narrowphaseId;
};


int32_t Collider::getPhysicsNarrowPhaseId() const
{
	return narrowphaseId;
};


const Position& Collider::getPosition() const
{
	return position;
};

const Velocity& Collider::getVelocity() const
{
	return velocity;
};


const Position& Collider::getOriginPosition() const
{
	if (originPositonNeedUpdate)
	{
		originPositon = { position.linear + (position.angular * -centerOfMassOffset), position.angular };
		originPositonNeedUpdate = false;
	}
	return originPositon;
};


const Vector2& Collider::getCenterOfMassOffset() const
{
	return centerOfMassOffset;
};


void Collider::setPosition(const Vector2& _v2)
{
	position.linear = _v2;
	invalidateOriginPositon();
};

void Collider::setPosition(const Quaternion& _q)
{
	position.angular = _q;
	invalidateOriginPositon();
};

void Collider::setPosition(const Position& _p)
{
	position = _p;
	invalidateOriginPositon();
};


void Collider::setVelocity(const Vector2& _v2)
{
	velocity.linear = _v2;
};

void Collider::setVelocity(float _a)
{
	velocity.angular = _a;
};

void Collider::setVelocity(const Velocity& _v)
{
	velocity = _v;
};



Vector2 Collider::getPartialPosition(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		Position p = integrate(pd->position, pd->velocity, timestep);

		return p.linear + p.angular * centerOfMassOffset; // origin as position??
	}

	Position p = integrate(position, velocity, timestep);
	return p.linear + p.angular * centerOfMassOffset; // origin as position??
};



Quaternion Collider::getPartialRotation(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		return integrate(pd->position.angular, pd->velocity.angular, timestep);
	}

	return integrate(position.angular, velocity.angular, timestep);
};


Position Collider::getPartialPositionAndRotation(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		Position pos = integrate(pd->position, pd->velocity, timestep);
		pos.linear += pos.angular * centerOfMassOffset; // origin as position??
		return pos;
	}

	Position pos = integrate(position, velocity, timestep);
	pos.linear += pos.angular * centerOfMassOffset; // origin as position??
	return pos;
};


Vector2 Collider::getPartialLinearVelocity(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		return pd->velocity.linear;
	}

	return velocity.linear;
};


float Collider::getPartialAngularVelocity(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		return pd->velocity.angular;
	}

	return velocity.angular;
};


Velocity Collider::getPartialVelocity(float timestep) const
{
	if (timestep < -EPSILON)
	{
		unsigned int i = 0;
		const PositionData *pd;
		do
			pd = &positionHistory.at(++i);
		while ((timestep += pd->timestep) < -EPSILON);

		return pd->velocity;
	}

	return velocity;
};


void Collider::addShape(ColliderShapePtr shape)
{
	shape->setParent(this);

	shapes.emplace_back(std::move(shape));
	shapes.shrink_to_fit();

	shapesTmpAABB.emplace_back();
	shapesTmpAABB.shrink_to_fit();
};


void Collider::updatePhysicsParams()
{
	float newMass(0.0f);
	Vector2 newCenterOfMass(0.0f);
	float newInertia(0.0f);

	bool infMassFlag = false;
	for (unsigned int i = 0, iMax = (unsigned int)shapes.size(); i < iMax; ++i)
	{
		float partialMass = shapes[i]->getMass();
		if (partialMass == (float)INFINITY)
		{
			infMassFlag = true;
			break;
		}
		newMass += partialMass;
		newCenterOfMass += shapes[i]->getCenterOfMass() * partialMass;
	}

	if (!infMassFlag)
	{
		for (unsigned int i = 0, iMax = (unsigned int)shapes.size(); i < iMax; ++i)
			newInertia += shapes[i]->getMMOI() + shapes[i]->getMass() * (shapes[i]->getCenterOfMass() - newCenterOfMass).lengthSq();
	}
	else
	{
		newMass = (float)INFINITY;
		newInertia = (float)INFINITY;
		newCenterOfMass = { 0.0f, 0.0f };
		unsigned int infCounter = 0;
		for (unsigned int i = 0, iMax = (unsigned int)shapes.size(); i < iMax; ++i)
		{
			float partialMass = shapes[i]->getMass();
			if (partialMass == (float)INFINITY)
			{
				newCenterOfMass += shapes[i]->getCenterOfMass();
				++infCounter;
			}
		}
		newCenterOfMass /= (float) infCounter;
	}

	mass = newMass;
	massRec = 1.0f / mass;
	inertia = newInertia;
	inertiaRec = 1.0f / inertia;

	// update offset
	position.linear += position.angular * (newCenterOfMass - centerOfMassOffset);
	centerOfMassOffset = newCenterOfMass;

	invalidateOriginPositon();
	clearPositionHistory();
};

/*
void Collider::progressTime(float _timestep)
{
	// store previous data
	pushPositionData(_timestep);

	// progress position and rotation

	position.linear = position.linear + _timestep * velocity.linear;
	position.angular = integrate(position.angular, velocity.angular, _timestep);

	// invalidate temporaries
	invalidateAABB();
	invalidateOriginPositon();
};
*/
/*
void Collider::update(float _timestep)
{
	for (unsigned int i = 0, iMax = (unsigned int)accelerators.size(); i < iMax; ++i)
		accelerators[i]->affect(*this, _timestep);
};
*/
/*
void Collider::progressAndUpdate(float _timestep)
{
	progressTime(_timestep);
	update(_timestep);
};
*/

/*
float Collider::rewind()
{
	return rollbackPositionData();
};


void Collider::rewind(float _timestep)
{
	while ((_timestep -= rollbackPositionData()) > EPSILON);

	if (_timestep < -EPSILON)
		progressTime(-_timestep);
};

*/

void Collider::applyImpulse(float time, const Vector2 &j, const Vector2 &offset)
{
	float timeRev = 0.0f, timeDiff = -time;
	while ((timeDiff += rollbackPositionData()) < -EPSILON);

	if (timeDiff > EPSILON)
	{
		pushPositionData(timeDiff); // store previous data
		position = integrate(position, velocity, timeDiff); // progress position and rotation
	}

	velocity.linear -= getMassReciprocal() * j;
	velocity.angular -= getInertiaReciprocal() * crossProduct(offset, j);

	position = integrate(position, velocity, time); // progress position and rotation

	pushPositionData(time); // store previous data
	position = integrate(position, velocity, time); // progress position and rotation
};


const AABB& Collider::getAABB() const
{
	if (AABBNeedUpdate)
	{
		tmpAABB.vMin = { (float)INFINITY, (float)INFINITY };
		tmpAABB.vMax = { -(float)INFINITY,  -(float)INFINITY };
		for (int i = 0, iMax = (int)shapes.size(); i < iMax; ++i)
		{
			shapesTmpAABB[i] = shapes[i]->getAABB();
			tmpAABB += shapesTmpAABB[i];
		}

		AABBNeedUpdate = false;
	}
	return tmpAABB;
};


const AABB& Collider::getShapeAABB(unsigned int index) const
{
	assert(index < shapes.size() && "index out of bounds");
	assert(!AABBNeedUpdate && "shape AABB not updated"); //might be unupdated?
	return shapesTmpAABB[index];
};


unsigned int Collider::getShapeCount() const
{
	return (unsigned int) shapes.size();
};


const ColliderShape& Collider::getShape(unsigned int index) const
{
	assert(index < shapes.size() && "index out of bounds");
	return *shapes[index];
};


MaterialID Collider::getShapeMaterial(unsigned int index) const
{
	assert(index < shapes.size() && "index out of bounds");
	return shapes[index]->getMaterialId();
};


/*
float Collider::getMaxRadiusFromCoM(unsigned int index) const
{
	assert(index < shapes.size() && "index out of bounds");
	return radiusFromCoM[index];
};
*/


float Collider::getMass() const
{
	return mass;
};


float Collider::getInertia() const
{
	return inertia;
};


const MassMatrix& Collider::getMassMx() const
{
	return massMx;
};


float Collider::getMassReciprocal() const
{
	return massRec;
};


float Collider::getInertiaReciprocal() const
{
	return inertiaRec;
};


const MassMatrix& Collider::getInvMassMx() const
{
	return invMassMx;
};


#if(DIRECT3D_VERSION == 0x0900)

	void Collider::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
	{
		for (int i = 0, iMax = (int)shapes.size(); i < iMax; ++i)
			shapes[i]->render(_pDevice, _vBuffer, _bufferLenght, _color ? _color : color);
	};

#endif
