#pragma once

#include <algorithm>
#include <map>
#include <vector>
#include <array>
#include <unordered_set>
#include <unordered_map>
#include "Constants.h"
#include "Math2D.h"
#include "Collider.h"
#include "ColliderShape.h"
#include "PhysicsBroadPhase.h"

#include "Constraint.h"
#include "PhysicsStructs.h"
#include "ImpulseStructs.h"
#include "Matrix.h"
#include "ColliderAffector.h"

#include "CharHash.h"
#include "Helper.h"

#include <d3d9.h>

#define DEFAULT_RESTITUTION_COEF	1.0f


class PhysicsNarrowPhase
{
private:
	//constexpr static float EPA_TOLERANCE = 0.0001f;
	float EPA_TOLERANCE = 0.001f;

	float COLLISION_BUFFER = 1.0f;


	typedef std::array<Vector2, 3> Simplex;

	struct SupportPair
	{
		Vector2 A, B;
	};


	struct CollisionData
	{
		float frameTOI;

		Collider *collider1;
		Collider *collider2;
		unsigned short shapeIndex1;
		unsigned short shapeIndex2;

		Vector2 contactPoint;
		Vector2 collisionNormal;

		//Features features;

		CollisionData() = default;
		CollisionData(float fTOI, Collider *c1, Collider *c2, unsigned short si1, unsigned short si2, const Vector2 &cp, const Vector2 &cn);
	};

	
	// colliders data
	std::unordered_map<const char *const, int32_t, CharHash<COLLIDER_ID_LEN>, CharComparer<COLLIDER_ID_LEN>> collidersMap;
	std::vector<ColliderSPtr> colliders;
	std::list<int32_t> collidersFreeId;


	// move all physics data into one array for optimisation
	// keep physics data 
	//std::vector<Position, Velocity, mass, massInv, inertia, inertiaInv> collidersVelocity;


	struct AffectorStruct
	{
		ColliderAffectorSPtr affectorPtr;

		AFFECTOR_SCOPE scope;

		std::vector<int32_t> colliderList;

		// ADD CACHE FORCE VECTOR FOR STATIC FORCES
	};

	std::unordered_map<const char *const, AffectorStruct, CharHash<AFFECTOR_ID_LEN>, CharComparer<COLLIDER_ID_LEN>> affectors;


	struct ConstraintData
	{
		int32_t b1;
		int32_t b2;
		ConstraintSPtr C;
		float lambda;

		ConstraintData(int32_t _b1, int32_t _b2, ConstraintSPtr _C, float _lambda) :
			b1(_b1), b2(_b2), C(_C), lambda(_lambda)
		{};
	};

	std::vector<ConstraintData> constraints;
	//std::vector<float> lambda;


	std::unordered_map<MaterialPair, float> restitutionCoefficients;

	PhysicsBroadPhase broadPhase;


protected:

	float getRestitutionCoefficient(const MaterialID& material1, const MaterialID& material2);

	void calculateTentativeVelocityAndPosition(float timestep);


	// distance / collision func
	float distance(const ColliderShape &o1, const ColliderShape &o2);
	bool distanceDirection(const ColliderShape &shape1, const Vector2 &position1, const Quaternion &rotation1, const ColliderShape &shape2, const Vector2 &position2, const Quaternion &rotation2, DirectionVector &distance);
	bool closestPoints(const ColliderShape &o1, const ColliderShape &o2, Vector2 &point1, Vector2 &point2);
	bool closestPoints(const Collider &o1, const Collider &o2, Vector2 &point1, Vector2 &point2);

	void closestFeaturesGJK2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2, DirectionVector &distance, Features *features = nullptr);
	bool intersectGJK2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2) const;

	bool getContactInformation(const Collider& c1, const Collider& c2, DirectionVector &distance, Vector2 &r1, Vector2 &r2, unsigned int &shapeIndex1, unsigned int &shapeIndex2);


	// used GJK functions after this point
	// continuos collision functions

	void intersectionDepth2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2, DirectionVector &depth, Features &closestFeatures, bool getDistance);

	float deepestPointSolverPtoP(const Collider &collider1, const ColliderShape &shape1, const Position& position1, const Collider &collider2, const ColliderShape &shape2, const Position& position2, DirectionVector &distance, Features &features, float timestep, float extTn);

	float deepestPointSolverStoP(const Collider &collider1, const ColliderShape &shape1, const Position& position1, const Collider &collider2, const ColliderShape &shape2, const Position& position2, DirectionVector &distance, Features &features, float timestep, float extTn);

	void deepestPointSolver(const Collider &collider1, unsigned int shapeIndex1, const Collider &collider2, unsigned int shapeIndex2, float timestep, float &tn, Features &features);


	void addTemporaryConstraint(ConstraintSPtr constraint, int32_t colliderId1, int32_t colliderId2);


public:

	void invalidateCollider(const Collider *_collider);

	void setBroadphaseMargin(float _boxMargin);

	int32_t addColider(ColliderSPtr _collider);
	void removeCollider(int32_t _id);
	void removeCollider(const char _name[COLLIDER_ID_LEN]);

	void addMaterialRestitutionCoefficient(MaterialID mId1, MaterialID mId2, float e);

	void registerAffector(ColliderAffectorSPtr affector, AFFECTOR_SCOPE scope);
	void unregisterAffector(const char affectorName[AFFECTOR_ID_LEN]);
	ColliderAffectorSPtr getAffector(const char affectorName[AFFECTOR_ID_LEN]) const;

	void registerAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], int32_t colliderId);
	void registerAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], const char colliderName[COLLIDER_ID_LEN]);

	void unregisterAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], int32_t colliderId);
	void unregisterAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], const char colliderName[COLLIDER_ID_LEN]);

	void addConstraint(ConstraintSPtr constraint, int32_t colliderId1, int32_t colliderId2);
	void addConstraint(ConstraintSPtr constraint, const char colliderName1[COLLIDER_ID_LEN], const char colliderName2[COLLIDER_ID_LEN]);

	void removeConstraint(const char _name[CONSTRAINT_ID_LEN]);
	void removeConstraint(ConstraintSPtr constraint);



	void update(const float timestep);

	void computeExternalForces(std::vector<Force>& Fext);
	void computeVelocityConstraints(float timestep);


	bool intersect(const Collider &collider, bool broadphaseOnly = false) const;

	bool continuosCollisionCheck(Collider& collider1, Collider& collider2, const float timestep, CollisionData& collisionData);

	void queryCollisions(const float timestep);


	inline void progressTime(float timestep)
	{
		calculateTentativeVelocityAndPosition(timestep);

		broadPhase.update();



		const CollisionList& collisionCandidates = broadPhase.queryCollisions(COLLISION_BUFFER);
		unsigned int c = (unsigned int)collisionCandidates.size();

		char collisionName[CONSTRAINT_ID_LEN] = "c_"; //len = 2

		for (unsigned int i = 0, c = (unsigned int)collisionCandidates.size(); i < c; ++i)
		{
			Collider &c1 = *collisionCandidates[i].first;
			Collider &c2 = *collisionCandidates[i].second;
			DirectionVector distance;
			Vector2 r1, r2;
			unsigned int shape1, shape2;

			if (!getContactInformation(c1, c2, distance, r1, r2, shape1, shape2))
				continue; // no contact


			unsigned int len = 2;
			len += itoas(c1.getPhysicsNarrowPhaseId(), &collisionName[len], CONSTRAINT_ID_LEN - len);
			collisionName[len++] = '_';
			len += itoas(c2.getPhysicsNarrowPhaseId(), &collisionName[len], CONSTRAINT_ID_LEN - len);
			if (len < CONSTRAINT_ID_LEN)
				collisionName[len] = '\0';

			float e = getRestitutionCoefficient(c1.getShapeMaterial(shape1), c2.getShapeMaterial(shape2));

			// collision constraint
			ConstraintSPtr collisionConstraint = std::make_shared<NormalConstraint>(collisionName, distance.direction, r1, r2, e);
			addTemporaryConstraint(collisionConstraint, c1.getPhysicsNarrowPhaseId(), c2.getPhysicsNarrowPhaseId());
		}






		// broadphase > check collisions
		// narrowphase > confirm collision
		// on each collision found check for other objects around with additional margin on size to get all possible hits into resolver
		


		// resolve collisions
		queryCollisions(timestep);

		// update speeds
		update(timestep);
	};


	inline void progressTime2(float timestep)
	{
		float timeRec = 1.0f / timestep;

		unsigned int n = (unsigned int)colliders.size();

		std::vector<Force> Fext(n);
		std::vector<Velocity> Vt(n);
		std::vector<Position> Xt(n);

		computeExternalForces(Fext);

		for (unsigned int i = 0; i < n; ++i)
		{
			if (colliders[i]->getMass() < (float)INFINITY)
			{
				const Velocity& v1 = colliders[i]->getVelocity();
				Velocity vd = colliders[i]->getInvMassMx() * Fext[i];
				//VMF[i] = v1 * timeRec + vd;
//MF[i] = colliders[i]->getInvMassMx * forces[i];
//VMF[i] = v1 * timeRec + MF[i];

				Vt[i] = v1 + vd * timestep;
//Vt[i] = v1 + MF[i] * timestep;
			}
			else // infinite mass, no velocity change possible
			{
				Vt[i] = colliders[i]->getVelocity();
				//VMF[i] = Vt[i] * timeRec;
			}

			const Position& x1 = colliders[i]->getPosition();
			Xt[i] = integrate(x1, Vt[i], timestep);
		}

	};
	







#if(DIRECT3D_VERSION == 0x0900)

	bool showDistance = false;

	void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght);

#endif

};
