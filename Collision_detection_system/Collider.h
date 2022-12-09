#pragma once

#include "Constants.h"
#include <cmath>
#include <vector>
#include "Vector2.h"
#include "Quaternion.h"
#include "ColliderShape.h"
#include "AABB.h"
#include "CircularBuffer.h"
#include "ColliderAffector.h"
#include "ImpulseStructs.h"


class Collider
{
private:
	char name[COLLIDER_ID_LEN];
	int32_t broadphaseId;
	int32_t narrowphaseId;

protected:



	Position position; // center of mass based, keep offset separate
	Velocity velocity;


	struct PositionData
	{
		float timestep;

		Position position;
		Velocity velocity;
	};
	CircularBuffer<PositionData, COLLIDER_STEP_HISTORY_SIZE> positionHistory;


	union
	{
		struct
		{
			float mass;
			float inertia; // MMOI - mass moment of inertia
		};
		MassMatrix massMx;
	};

	union
	{
		struct
		{
			float massRec;
			float inertiaRec;
		};
		MassMatrix invMassMx;
	};

	//
	Vector2 centerOfMassOffset;

	mutable bool originPositonNeedUpdate;
	mutable Position originPositon;


	typedef std::vector<ColliderShapePtr> ShapesContainer;
	ShapesContainer shapes;
	//std::vector<float> radiusFromCoM;

	// temporaries with update flags
	mutable bool AABBNeedUpdate;
	mutable std::vector<AABB> shapesTmpAABB;
	mutable AABB tmpAABB;


	void invalidateAABB();
	void invalidateOriginPositon();

	void pushPositionData(float timestep);
	float rollbackPositionData();

	void clearPositionHistory();


	std::vector<ColliderAffectorSPtr> affectors;


public:

	Collider();
	~Collider();

	void setName(const char _name[COLLIDER_ID_LEN]);
	const char* getName() const;

	void setPhysicsBroadPhaseId(int32_t _broadphaseId);
	int32_t getPhysicsBroadPhaseId() const;

	void setPhysicsNarrowPhaseId(int32_t _broadphaseId);
	int32_t getPhysicsNarrowPhaseId() const;

	const Position& getPosition() const;
	const Velocity& getVelocity() const;
	const Position& getOriginPosition() const;
	const Vector2& getCenterOfMassOffset() const;

	void setPosition(const Vector2& _v2);
	void setPosition(const Quaternion& _q);
	void setPosition(const Position& _p);

	void setVelocity(const Vector2& _v2);
	void setVelocity(float _a);
	void setVelocity(const Velocity& _v);



	Vector2 getPartialPosition(float timestep) const;
	Quaternion getPartialRotation(float timestep) const;
	//void getPartialPositionAndRotation(float timestep, Position& _position) const;
	Position getPartialPositionAndRotation(float timestep) const;

	Vector2 getPartialLinearVelocity(float timestep) const;
	float getPartialAngularVelocity(float timestep) const;
	Velocity getPartialVelocity(float timestep) const;

	void addShape(ColliderShapePtr shape);
	void updatePhysicsParams();

	//void progressTime(float _timestep);
	//void update(float _timestep);
	//void progressAndUpdate(float _timestep);
	//float rewind();
	//void rewind(float _timestep);
	// time off current position
	void applyImpulse(float time, const Vector2 &j, const Vector2 &offset);

	const AABB& getAABB() const;
	const AABB& getShapeAABB(unsigned int index) const;

	unsigned int getShapeCount() const;
	const ColliderShape& getShape(unsigned int index) const;
	MaterialID getShapeMaterial(unsigned int index) const;

	//float getMaxRadiusFromCoM(unsigned int index) const;

	float getMass() const;
	float getInertia() const;
	const MassMatrix& getMassMx() const;

	float getMassReciprocal() const;
	float getInertiaReciprocal() const;
	const MassMatrix& getInvMassMx() const;



#if(DIRECT3D_VERSION == 0x0900)

public:
	// debug
	DWORD color;
	void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color = 0x00000000);

#endif

};

typedef std::shared_ptr<Collider> ColliderSPtr;
