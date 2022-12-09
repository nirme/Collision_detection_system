#pragma once


/*

void PhysicsNarrowPhase::queryCollisions(const float timestep)
{
	const CollisionList& collisionCandidates = broadPhase.queryCollisions();
	if (!collisionCandidates.size()) // no collisions found
		return;

	std::multimap<float, ColisionData> collisions;

	for (unsigned int i = 0, iMax = (unsigned int)collisionCandidates.size(); i < iMax; ++i)
	{
		Collider &collider1 = *collisionCandidates[i].first;
		Collider &collider2 = *collisionCandidates[i].second;

		Vector2 pos1 = collider1.getPosition();
		Vector2 pos2 = collider2.getPosition();
		Quaternion rot1 = collider1.getRotation();
		Quaternion rot2 = collider2.getRotation();

		DirectionVector collisionVector({}, (float)INFINITY);
		Features collidingFeatures;
		unsigned short shapeIndex1;
		unsigned short shapeIndex2;

		for (unsigned int csi1 = 0, csMax1 = collider1.getShapeCount(); csi1 < csMax1; ++csi1)
		{
			for (unsigned int csi2 = 0, csMax2 = collider2.getShapeCount(); csi2 < csMax2; ++csi2)
			{
				DirectionVector tmpDepth;
				Features tmpFeatures;

				// test shapes AABB, those shoud be updated by broadphase already
				if (!isOverlapping(collider1.getShapeAABB(csi1), collider2.getShapeAABB(csi2)))
					continue;

				closestFeaturesGJK2(collider1.getShape(csi1), pos1, rot1, collider2.getShape(csi2), pos2, rot2, tmpDepth, tmpFeatures);
				if (tmpDepth.magnitude < collisionVector.magnitude)
				{
					collisionVector = tmpDepth;
					collidingFeatures = tmpFeatures;
					shapeIndex1 = csi1;
					shapeIndex2 = csi2;
				}
			}
		}

		// no collision
		if (collisionVector.magnitude > COLLISION_DISTANCE_EPSILON)
			continue;

		//exact collision distance means collision at end of frame so TOI = 1
		float TOI = 0.0f;
		float frameTOI = 0.0f;

		// calculate TOI
		while (collisionVector.magnitude < -COLLISION_DISTANCE_EPSILON)
		{
			deepestPointSolver(collider1, shapeIndex1, collider2, shapeIndex2, timestep, TOI);

			frameTOI = timestep * (TOI - 1.0f);
			collider1.getPartialPositionAndRotation(frameTOI, pos1, rot1);
			collider2.getPartialPositionAndRotation(frameTOI, pos2, rot2);

			//collisionVector.magnitude = COLLISION_DISTANCE_EPSILON;
			collisionVector.magnitude = (float)INFINITY;
			for (unsigned int csi1 = 0, csMax1 = collider1.getShapeCount(); csi1 < csMax1; ++csi1)
			{
				for (unsigned int csi2 = 0, csMax2 = collider2.getShapeCount(); csi2 < csMax2; ++csi2)
				{
					DirectionVector tmpDepth;
					Features tmpFeatures;

					const ColliderShape &shape1 = collider1.getShape(csi1);
					const ColliderShape &shape2 = collider2.getShape(csi2);

					if (!shape1.getAABB(pos1, rot1).isOverlapping(shape2.getAABB(pos2, rot2)))
						continue;

					closestFeaturesGJK2(collider1.getShape(csi1), pos1, rot1, collider2.getShape(csi2), pos2, rot2, tmpDepth, tmpFeatures);

					//if (tmpDepth.magnitude <= COLLISION_DISTANCE_EPSILON && tmpDepth.magnitude < collisionVector.magnitude)
					if (tmpDepth.magnitude < collisionVector.magnitude)
					{
						collisionVector = tmpDepth;
						collidingFeatures = tmpFeatures;
						shapeIndex1 = csi1;
						shapeIndex2 = csi2;
					}
				}
			}
		}

		collisions.emplace(std::piecewise_construct,
			std::forward_as_tuple(frameTOI),
			std::forward_as_tuple(frameTOI, &collider1, &collider2, shapeIndex1, shapeIndex2, collidingFeatures));
	}


	std::unordered_set<Collider*> updatedColliders;

	// resolve impacts
	for (auto it = collisions.begin(), itEnd = collisions.end(); it != itEnd; ++it)
	{
		ColisionData &cd = it->second;
		Features &collidingFeatures = cd.features;

		Vector2 contactNormal, r1, r2, Vr;

		Vector2 V1 = cd.collider1->getPartialLinearSpeed(cd.frameTOI);
		float w1 = cd.collider1->getPartialAngularSpeed(cd.frameTOI);
		Vector2 V2 = cd.collider2->getPartialLinearSpeed(cd.frameTOI);
		float w2 = cd.collider2->getPartialAngularSpeed(cd.frameTOI);

		Vector2 contactPoint;
		FeatureHash fh = FEATURE_HASH(collidingFeatures.f1.type, collidingFeatures.f2.type);

		/*
		if (fh != FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT))
		{
			if (fh == FEATURE_HASH(Feature::FT_POINT, Feature::FT_SEGMENT))
			{
				contactPoint = pointOnSegment(collidingFeatures.f1.point, pointOnSegment(collidingFeatures.f2.segmentP1, collidingFeatures.f2.segmentP2));
				contactNormal = -normalAt(collidingFeatures.f2.segmentP1, collidingFeatures.f2.segmentP2, cd.collider1->getPosition()).normalize();
			}
			else if (fh == FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_POINT))
			{
				contactPoint = pointOnSegment(pointOnSegment(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2), collidingFeatures.f2.point);
				contactNormal = normalAt(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2, cd.collider2->getPosition()).normalize();
			}
			else if (fh == FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_SEGMENT))
			{
				Vector2 tg = collidingFeatures.f1.segmentP1 - collidingFeatures.f1.segmentP2;

				float dTgF1P1 = dotProduct(tg, tg);
				float dTgF2P2 = dotProduct(tg, collidingFeatures.f2.segmentP2 - collidingFeatures.f1.segmentP2);
				Vector2 seg1 = dTgF1P1 < dTgF2P2 ? collidingFeatures.f1.segmentP1 : collidingFeatures.f2.segmentP2;

				float dTgF2P1 = dotProduct(tg, collidingFeatures.f2.segmentP1 - collidingFeatures.f1.segmentP2);
				Vector2 seg2 = dTgF2P1 >= 0.0f ? collidingFeatures.f2.segmentP1 : collidingFeatures.f1.segmentP2;

				contactPoint = pointOnSegment(seg1, seg2);
				contactNormal = normalAt(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2, cd.collider2->getPosition()).normalize();
			}

			r1 = contactPoint - cd.collider1->getCenterOfMass();
			r2 = contactPoint - cd.collider2->getCenterOfMass();
			Vr = (V2 + crossProduct(w2, r2)) - (V1 + crossProduct(w1, r1));
		}
		else // fh == FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT) - lowest probability
		{
			contactPoint = pointOnSegment(collidingFeatures.f1.point, collidingFeatures.f2.point);
			r1 = contactPoint - cd.collider1->getCenterOfMass();
			r2 = contactPoint - cd.collider2->getCenterOfMass();
			Vr = (V2 + crossProduct(w2, r2)) - (V1 + crossProduct(w1, r1));
			contactNormal = Vr.normalize();
		}
	*	/

switch (fh)
{
case FEATURE_HASH(Feature::FT_POINT, Feature::FT_SEGMENT):
{
	contactNormal = -normalAt(collidingFeatures.f2.segmentP1, collidingFeatures.f2.segmentP2, cd.collider1->getPosition()).normalize();
	contactPoint = pointOnSegment(collidingFeatures.f1.point, pointOnSegment(collidingFeatures.f2.segmentP1, collidingFeatures.f2.segmentP2));
	break;
}
case FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_POINT):
{
	contactNormal = normalAt(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2, cd.collider2->getPosition()).normalize();
	contactPoint = pointOnSegment(pointOnSegment(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2), collidingFeatures.f2.point);
	break;
}
case FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_SEGMENT):
{
	Vector2 tg = collidingFeatures.f1.segmentP1 - collidingFeatures.f1.segmentP2;

	float dTgF1P1 = dotProduct(tg, tg);
	float dTgF2P2 = dotProduct(tg, collidingFeatures.f2.segmentP2 - collidingFeatures.f1.segmentP2);
	Vector2 seg1 = dTgF1P1 < dTgF2P2 ? collidingFeatures.f1.segmentP1 : collidingFeatures.f2.segmentP2;

	float dTgF2P1 = dotProduct(tg, collidingFeatures.f2.segmentP1 - collidingFeatures.f1.segmentP2);
	Vector2 seg2 = dTgF2P1 >= 0.0f ? collidingFeatures.f2.segmentP1 : collidingFeatures.f1.segmentP2;

	contactPoint = pointOnSegment(seg1, seg2);
	contactNormal = normalAt(collidingFeatures.f1.segmentP1, collidingFeatures.f1.segmentP2, cd.collider2->getPosition()).normalize();
	break;
}
case FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT):
{
	contactPoint = pointOnSegment(collidingFeatures.f1.point, collidingFeatures.f2.point);

	r1 = contactPoint - cd.collider1->getCenterOfMass();
	r2 = contactPoint - cd.collider2->getCenterOfMass();
	Vr = (V2 + crossProduct(w2, r2)) - (V1 + crossProduct(w1, r1));

	contactNormal = Vr.normalize();
	break;
}
}

// 
if (fh != FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT))
{
	r1 = contactPoint - cd.collider1->getCenterOfMass();
	r2 = contactPoint - cd.collider2->getCenterOfMass();
	Vr = (V2 + crossProduct(w2, r2)) - (V1 + crossProduct(w1, r1));
}


auto eIt = restitutionCoefficients.find(MaterialPair(cd.collider1->getShapeMaterial(cd.shapeIndex1), cd.collider2->getShapeMaterial(cd.shapeIndex2)));
float e = DEFAULT_RESTITUTION_COEF;
if (eIt != restitutionCoefficients.end())
e = eIt->second;


//                               -(1+e) * Vr * n
// jr = -------------------------------------------------------------------------
//       m1^-1 + m2^-1 + ((I1^-1 * (r1 x n) x r1) + (I2^-1 * (r2 x n) x r2)) * n

float jrNum = -(1.0f + e) * dotProduct(Vr, contactNormal);

Vector2 cpdv = crossProductDouble(r1, contactNormal, r1) * cd.collider1->getInertiaReciprocal();
cpdv += crossProductDouble(r2, contactNormal, r2) * cd.collider2->getInertiaReciprocal();

float jrDenom = dotProduct(cpdv, contactNormal);
jrDenom += cd.collider1->getMassReciprocal();
jrDenom += cd.collider2->getMassReciprocal();

float jr = jrNum / jrDenom;
Vector2 vJr = jr * contactNormal;


cd.collider1->applyCollisionImpulse(cd.frameTOI, vJr, r1);
cd.collider2->applyCollisionImpulse(cd.frameTOI, -vJr, r2);


updatedColliders.emplace(cd.collider1);
updatedColliders.emplace(cd.collider2);
	}

};

*/















/*

bool intersect(const ColliderShape &o1, const ColliderShape &o2, Simplex &simplex)
{
	// first point
	Vector2 d(o2.getWorldPosition() - o1.getWorldPosition());
	d = d.normalize();
	simplex[0] = o1.getWorldSupport(d) - o2.getWorldSupport(-d);

	// second point
	d = -simplex[0];
	d = d.normalize();
	simplex[1] = o1.getWorldSupport(d) - o2.getWorldSupport(-d);
	// didn't pass center
	if (dotProduct(d, simplex[1]) < 0.0f)
		return false;

	unsigned int step = 0;
	// repeatedly look for third point reusing 2 closes to center
	while (++step <= GJK_STEPS_LIMIT)
	{
		d = normalAtOrigin(simplex[0], simplex[1]);
		if (d.isZero())
		{
			Vector2 s01 = simplex[1] - simplex[0];
			float dist = dotProduct(s01, -simplex[0]);

			// origin lies before s0
			if (dist <= (EPSILON * EPSILON))
				d = -simplex[0];
			// origin lies after s1
			else if (dist - s01.lengthSq() >= -(EPSILON * EPSILON))
				d = -simplex[1];
			// origin lies on the segment
			else
				d = normalVectorCCW(s01);
		}
		d = d.normalize();

		simplex[2] = o1.getWorldSupport(d) - o2.getWorldSupport(-d);

		// didn't pass center
		if (dotProduct(d, simplex[2]) < 0.0f)
			return false;


		// center lies outside next to line simplex[1] and [2], remove simplex[0] //////////////////////////// BAD!!!!!
		Vector2 dn = normalAtOrigin(simplex[1], simplex[2]);
		if (dotProduct(dn, simplex[0]) < EPSILON)
		{
			simplex[0] = simplex[2];
			continue;
		}

		// center lies outside next to line simplex[2] and [0], remove simplex[1]
		dn = normalAtOrigin(simplex[2], simplex[0]);
		if (dotProduct(dn, simplex[1]) < EPSILON)
		{
			simplex[1] = simplex[2];
			continue;
		}

		// center lies inside
		return true;
	}

	assert(false && "GJK test passed max number of steps");
	return false;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	SIMPLEX POINTS CAN BE "SORTED" TO ALWAYS BE IN SPECIFIC ORDER
	//	!!!!! CHANGE IT TO LIMIT EPA PART
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};


DirectionVector penetrationDepth(const ColliderShape &o1, const ColliderShape &o2, const Simplex &simplex)
{
	// Expanding Polytope Algorithm
	std::vector<Vector2> polygon(simplex.begin(), simplex.end());
	std::vector<DirectionVector> edges(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
		std::swap(polygon[1], polygon[2]);

	// calc normals and distances
	for (int i = 0; i < 3; ++i)
	{
		edges[i].direction = normalVectorCCW(polygon[(i + 1) % 3] - polygon[i]).normalize();
		edges[i].magnitude = dotProduct(edges[i].direction, polygon[i]);
	}

	unsigned int step = 0;
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		int edgeId = 0;
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = o1.getWorldSupport(edges[edgeId].direction) - o2.getWorldSupport(-edges[edgeId].direction);

		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
			return { edges[edgeId].direction, newMagnitude };


		// insert point
		polygon.insert(polygon.begin() + edgeId + 1, point);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + edgeId + 1, 1, {});

		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(false && "EPA test passed max number of steps");
	return { {0.0f}, 0.0f };
};


bool intersectTest(const Collider &o1, const Collider &o2, DirectionVector &penetration)
{
	penetration = { {0.0f, 0.0f }, (float)-INFINITY };

	Simplex simplex;
	bool intersectionDetected = false;

	for (unsigned int i = 0, o1sMax = o1.getShapeCount(); i < o1sMax; ++i)
	{
		for (unsigned int j = 0, o2sMax = o2.getShapeCount(); j < o2sMax; ++j)
		{
			if (intersect(o1.getShape(i), o2.getShape(j), simplex))
			{
				intersectionDetected = true;
				DirectionVector testPenetration = penetrationDepth(o1.getShape(i), o2.getShape(j), simplex);

				if (testPenetration.magnitude > penetration.magnitude)
				{
					penetration = testPenetration;
				}
			}
		}
	}

	// should probably pass shape iterators for collision resolver
	return intersectionDetected;
};


bool closestFeaturesGJK(const Collider &o1, unsigned int shapeIndex1, const Collider &o2, unsigned int shapeIndex2, float timestep, DirectionVector &distance)
{
	Vector2 pos1, pos2;
	Quaternion rot1, rot2;
	o1.getPartialPositionAndRotation(timestep, pos1, rot1);
	o2.getPartialPositionAndRotation(timestep, pos2, rot2);
	const ColliderShape &shape1 = o1.getShape(shapeIndex1);
	const ColliderShape &shape2 = o2.getShape(shapeIndex2);

	Simplex simplex;
	unsigned int simplexPoints;
	unsigned int step = 0;

	Vector2 d = (shape2.getPosition(pos2, rot2) - shape1.getPosition(pos1, rot1)).normalize();
	simplex[0] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);

	d = -simplex[0].normalize();
	simplex[1] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);

	if (dotProduct(d, simplex[1]) < 0.0f)
	{
		simplexPoints = 2;
		goto Distance_GJK;
	}

	while (++step <= GJK_STEPS_LIMIT)
	{
		d = normalAtOrigin(simplex[0], simplex[1]);
		if (d.isZero())
		{
			Vector2 s01 = simplex[1] - simplex[0];
			float dist = dotProduct(s01, -simplex[0]);

			// origin lies before s0
			if (dist <= (EPSILON * EPSILON))
				d = -simplex[0];
			// origin lies after s1
			else if (dist - s01.lengthSq() >= -(EPSILON * EPSILON))
				d = -simplex[1];
			// origin lies on the segment
			else
				d = normalVectorCCW(s01);
		}
		d = d.normalize();

		simplex[2] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);
		if (dotProduct(d, simplex[2]) < 0.0f)
		{
			simplexPoints = 3;
			goto Distance_GJK;
		}

		if (dotProduct(normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
			simplex[0] = simplex[2];
		else if (dotProduct(normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
			simplex[1] = simplex[2];
		else
			goto Depth_EPA;
	}

	assert(false && "GJK test passed max number of steps");



Distance_GJK:

	bool singlePointFlag;

	if (simplexPoints == 2)
	{
		singlePointFlag = true;
		simplex[0] = simplex[1];
		d = -simplex[0];
	}
	else // simplexPoints == 3
	{
		Vector2 s02 = simplex[2] - simplex[0];
		Vector2 s12 = simplex[2] - simplex[1];

		if (-dotProduct(s02, simplex[0]) < s02.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-s02.y, s02.x) * -crossProduct(s02, simplex[0]); // crossProduct(AC, 0 - A)
			simplex[1] = simplex[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
		}
		else
		{
			singlePointFlag = true;
			simplex[0] = simplex[2];
			d = -simplex[0];
		}
	}

	step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		if (d.lengthSq() < EPSILON*EPSILON)
		{
			distance = { {0.0f, 0.0f}, 0.0f };
			return true;
		}

		d = d.normalize();
		simplex[2] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);

		float dS2 = dotProduct(d, simplex[2]);
		float dS0 = dotProduct(d, simplex[0]);
		//if (dC - dA < std::abs(dC * EPSILON))
		if (dS2 - dS0 < EPSILON)
		{
			distance.direction = d;
			distance.magnitude = !singlePointFlag ? -dS0 : simplex[2].length();
			return false;
		}

		Vector2 s02 = simplex[2] - simplex[0];
		Vector2 s12 = simplex[2] - simplex[1];

		if (-dotProduct(s02, simplex[0]) < s02.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-s02.y, s02.x) * -crossProduct(s02, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[1] = simplex[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
		}
		else
		{
			singlePointFlag = true;
			d = -simplex[2];
			simplex[0] = simplex[2];
		}
	}

	assert(false && "GJK distance test passed max number of steps");



Depth_EPA:

	std::vector<Vector2> polygon(simplex.begin(), simplex.end());
	std::vector<DirectionVector> edges(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
		std::swap(polygon[1], polygon[2]);

	// calc normals and distances
	for (int i = 0; i < 3; ++i)
	{
		edges[i].direction = normalVectorCCW(polygon[(i + 1) % 3] - polygon[i]).normalize();
		edges[i].magnitude = dotProduct(edges[i].direction, polygon[i]);
	}

	step = 0;
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		int edgeId = 0;
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = shape1.getSupport(pos1, rot1, edges[edgeId].direction) - shape2.getSupport(pos2, rot2, -edges[edgeId].direction);

		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
		{
			distance = { edges[edgeId].direction, -newMagnitude };
			return true;
		}


		// insert point
		polygon.insert(polygon.begin() + edgeId + 1, point);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + edgeId + 1, 1, {});

		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(false && "EPA test passed max number of steps");

	return true;
};

bool closestFeaturesGJK(const ColliderShape &shape1, const Vector2 &position1, const Quaternion &rotation1, const ColliderShape &shape2, const Vector2 &position2, const Quaternion &rotation2, DirectionVector &distance, Features &features)
{
	Vector2 supA1, supA2, supB1, supB2, supC1, supC2;
	Vector2 A, B, C, d, dTmp;

	dTmp = shape2.getPosition(position2, rotation2) - shape1.getPosition(position1, rotation1);
	dTmp = dTmp.normalize();
	A = (supA1 = shape1.getSupport(position1, rotation1, dTmp)) - (supA2 = shape2.getSupport(position2, rotation2, -dTmp));

	d = -A.normalize();
	B = (supB1 = shape1.getSupport(position1, rotation1, d)) - (supB2 = shape2.getSupport(position2, rotation2, -d));

	Vector2 AB = B - A;
	float projDistSq = -dotProduct(AB, A); // dotProduct(segmentVect, 0 - A)

	if (projDistSq <= 0.0f)
	{
		distance.direction = dTmp;
		distance.magnitude = dotProduct(dTmp, A);
		features.f1.set(supA1);
		features.f1.set(supA2);
		return true;
	}

	if (projDistSq >= AB.lengthSq())
	{
		distance.direction = d;
		distance.magnitude = dotProduct(d, B);
		features.f1.set(supB1);
		features.f1.set(supB2);
		return true;
	}

	// A-B
	bool singlePointFlag = false;
	d = Vector2(-AB.y, AB.x) * -crossProduct(AB, A); // crossProduct(AB, 0 - A)

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{

		if (d.lengthSq() < EPSILON*EPSILON)
			return false;

		d = d.normalize();
		C = (supC1 = shape1.getSupport(position1, rotation1, d)) - (supC2 = shape2.getSupport(position2, rotation2, -d));

		float dC = dotProduct(d, C), dA = dotProduct(d, A);
		// d*c >= d*a // keep d normalized, measure against relative epsilon to stop false negatives
		// C didnt get closer than previous point
		// point is on the line or A if singlePointFlag
		//if (dC - dotProduct(d, A) < std::abs(dC * EPSILON))
		if (dC - dA < EPSILON)
		{
			if (!singlePointFlag)
			{
				if ((supA1 - supB1).lengthSq() > EPSILON)
					features.f1.set(supA1, supB1);
				else
					features.f1.set(supA1);

				if ((supA2 - supB2).lengthSq() > EPSILON)
					features.f2.set(supA2, supB2);
				else
					features.f2.set(supA2);

				distance.direction = d;
				distance.magnitude = dA;
			}
			else
			{
				distance.direction = d;
				distance.magnitude = dA;
				features.f1.set(supA1);
				features.f1.set(supA2);
			}

			return true;
		}


		if (!singlePointFlag)
		{
			float crossAAB = crossProduct(A, B - A);
			if (crossAAB * crossProduct(B, C - B) >= 0.0f && crossAAB * crossProduct(C, A - C) >= 0.0f)
				return false;
		}

		Vector2 AC = C - A;
		Vector2 BC = C - B;

		if (-dotProduct(AC, A) < AC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-AC.y, AC.x) * -crossProduct(AC, A); // crossProduct(AB, 0 - A)
			B = C;
			supB1 = supC1;
			supB2 = supC2;
		}
		else if (-dotProduct(BC, B) < BC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(BC.y, -BC.x) * crossProduct(BC, A); // crossProduct(AB, 0 - A)
			A = C;
			supA1 = supC1;
			supA2 = supC2;
		}
		else
		{
			singlePointFlag = true;
			d = -C;
			A = C;
			supA1 = supC1;
			supA2 = supC2;
		}
	}

	return false;
};

bool deepestFeaturesGJK(const ColliderShape &shape1, const Vector2 &position1, const Quaternion &rotation1, const ColliderShape &shape2, const Vector2 &position2, const Quaternion &rotation2, DirectionVector &distance, Features &features)
{
	std::vector<Vector2> simplex(3);
	unsigned int step = 0;

	Vector2 d = (shape2.getPosition(position2, rotation2) - shape1.getPosition(position1, rotation1)).normalize();
	simplex[0] = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);

	d = -simplex[0].normalize();
	simplex[1] = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);

	if (dotProduct(d, simplex[1]) < 0.0f)
		return false;

	while (++step <= GJK_STEPS_LIMIT)
	{
		d = normalAtOrigin(simplex[0], simplex[1]);
		if (std::abs(d.x) < EPSILON && std::abs(d.y) < EPSILON)
			//if (std::abs((d.x + d.y) + (d.x * d.y)) < EPSILON)
			d = normalVectorCCW(simplex[1]);
		d = d.normalize();

		simplex[2] = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);
		if (dotProduct(d, simplex[2]) < 0.0f)
			return false;

		if (dotProduct(normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
			simplex[0] = simplex[2];
		else if (dotProduct(normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
			simplex[1] = simplex[2];
		else
			break;
	}

	assert(step <= GJK_STEPS_LIMIT && "GJK test passed max number of steps");


	std::vector<Vector2> &polygon = simplex;
	std::vector<DirectionVector> edges(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
		std::swap(polygon[1], polygon[2]);

	// calc normals and distances
	for (int i = 0; i < 3; ++i)
	{
		edges[i].direction = normalVectorCCW(polygon[(i + 1) % 3] - polygon[i]).normalize();
		edges[i].magnitude = dotProduct(edges[i].direction, polygon[i]);
	}

	step = 0;
	int edgeId = 0; // closest edge id
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = shape1.getSupport(position1, rotation1, edges[edgeId].direction) - shape2.getSupport(position2, rotation2, -edges[edgeId].direction);

		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
		{
			features.f1 = shape1.getLocalClosestFeature((-rotation1) * edges[edgeId].direction);
			features.f2 = shape2.getLocalClosestFeature((-rotation2) * (-edges[edgeId].direction));
			distance = { edges[edgeId].direction, -newMagnitude };
			return true;
		}


		unsigned int insIndex = edgeId + 1;
		// insert point
		polygon.insert(polygon.begin() + insIndex, point);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + insIndex, 1, {});

		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(false && "EPA test passed max number of steps");

	return true;
};

bool intersectGJK(const ColliderShape &shape1, const ColliderShape &shape2, DirectionVector &distance)
{
	Simplex simplex;

	Vector2 d = (shape2.getWorldPosition() - shape1.getWorldPosition()).normalize();
	simplex[0] = shape1.getWorldSupport(d) - shape2.getWorldSupport(-d);

	d = -simplex[0].normalize();
	simplex[1] = shape1.getWorldSupport(d) - shape2.getWorldSupport(-d);

	if (dotProduct(d, simplex[1]) < 0.0f)
		return false;

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		d = normalAtOrigin(simplex[0], simplex[1]);
		if (std::abs(d.x) < EPSILON && std::abs(d.y) < EPSILON)
			//if (std::abs((d.x + d.y) + (d.x * d.y)) < EPSILON)
			d = normalVectorCCW(simplex[1]);
		d = d.normalize();

		simplex[2] = shape1.getWorldSupport(d) - shape2.getWorldSupport(-d);
		if (dotProduct(d, simplex[2]) < 0.0f)
			return false;

		if (dotProduct(normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
			simplex[0] = simplex[2];
		else if (dotProduct(normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
			simplex[1] = simplex[2];
		else
			goto Depth_EPA;
	}

	assert(false && "GJK test passed max number of steps");

Depth_EPA:

	std::vector<Vector2> polygon(simplex.begin(), simplex.end());
	std::vector<DirectionVector> edges(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
		std::swap(polygon[1], polygon[2]);

	// calc normals and distances
	for (int i = 0; i < 3; ++i)
	{
		edges[i].direction = normalVectorCCW(polygon[(i + 1) % 3] - polygon[i]).normalize();
		edges[i].magnitude = dotProduct(edges[i].direction, polygon[i]);
	}

	step = 0;
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		int edgeId = 0;
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = shape1.getWorldSupport(edges[edgeId].direction) - shape2.getWorldSupport(-edges[edgeId].direction);
		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
		{
			distance = { edges[edgeId].direction, -newMagnitude };
			return true;
		}

		unsigned int insIndex = edgeId + 1;
		// insert point
		polygon.insert(polygon.begin() + insIndex, point);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + insIndex, 1, {});
		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(false && "EPA test passed max number of steps");
	return false;
};


bool intersectionDepth(const ColliderShape &shape1, const Vector2 &pos1, const Quaternion &rot1, const ColliderShape &shape2, const Vector2 &pos2, const Quaternion &rot2, DirectionVector &depth)
{
	//Simplex simplex;
	std::vector<Vector2> simplex(EPA_POLYGON_RESERVED_SIZE);
	simplex.resize(3);

	Vector2 d = (pos2 - pos1).normalize();
	simplex[0] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);

	d = -simplex[0].normalize();
	simplex[1] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);

	if (dotProduct(d, simplex[1]) < 0.0f)
		return false;

	// additional step in case of d being 0 before we got 3rd point of the simplex
	d = normalAtOrigin(simplex[0], simplex[1]);
	if (d.isZero())
		d = normalVectorCCW(simplex[1]);

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		d = d.normalize();

		simplex[2] = shape1.getSupport(pos1, rot1, d) - shape2.getSupport(pos2, rot2, -d);
		if (dotProduct(d, simplex[2]) < 0.0f)
			return false;

		if (dotProduct(normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
			simplex[0] = simplex[2];
		else if (dotProduct(normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
			simplex[1] = simplex[2];
		else
			goto Depth_EPA;

		d = normalAtOrigin(simplex[0], simplex[1]);
	}

	assert(false && "GJK test passed max number of steps");

Depth_EPA:

	std::vector<Vector2> &polygon = simplex;
	std::vector<DirectionVector> edges(EPA_POLYGON_RESERVED_SIZE);
	edges.resize(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
		std::swap(polygon[1], polygon[2]);

	// calc normals and distances
	for (int i = 0; i < 3; ++i)
	{
		edges[i].direction = normalVectorCCW(polygon[(i + 1) % 3] - polygon[i]).normalize();
		edges[i].magnitude = dotProduct(edges[i].direction, polygon[i]);
	}

	step = 0;
	// CHECK THE AMOUNT OF STEPS ACTUALY USED IN REAL LIFE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		int edgeId = 0;
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = shape1.getSupport(pos1, rot1, edges[edgeId].direction) - shape2.getSupport(pos2, rot2, -edges[edgeId].direction);
		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
		{
			depth = { edges[edgeId].direction, newMagnitude };
			return true;
		}

		unsigned int insIndex = edgeId + 1;
		// insert point
		polygon.insert(polygon.begin() + insIndex, point);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + insIndex, 1, {});
		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(false && "EPA test passed max number of steps");

	// return closest edge in case we passed max steps, it should be close to real depth anyway
	int edgeId = 0;
	for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
		if (edges[i].magnitude < edges[edgeId].magnitude)
			edgeId = i;

	depth = { edges[edgeId].direction, -edges[edgeId].magnitude };

	return true;
};
*/
