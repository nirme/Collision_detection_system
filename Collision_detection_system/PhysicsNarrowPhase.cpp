#include "PhysicsNarrowPhase.h"


PhysicsNarrowPhase::CollisionData::CollisionData(float fTOI, Collider *c1, Collider *c2, unsigned short si1, unsigned short si2, const Vector2 &cp, const Vector2 &cn) : 
	frameTOI(fTOI),
	collider1(c1),
	collider2(c2),
	shapeIndex1(si1),
	shapeIndex2(si2),
	contactPoint(cp),
	collisionNormal(cn)
{};


void PhysicsNarrowPhase::closestFeaturesGJK2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2, DirectionVector &distance, Features *features)
{
	std::vector<Vector2> simplex(3);
	std::array<std::pair<Vector2, Vector2>, 3> supports;
	unsigned int simplexPoints;
	unsigned int step = 0;

	Vector2 d = (shape2.getPosition(pos2.linear, pos2.angular) - shape1.getPosition(pos1.linear, pos1.angular)).normalize();
	simplex[0] = (supports[0].first = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[0].second = shape2.getSupport(pos2.linear, pos2.angular, -d));

	// d could be 0 if circles are just touching, in that case just flip the direction
	d = !simplex[0].isZero() ? -simplex[0].normalize() : -d;

	simplex[1] = (supports[1].first = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[1].second = shape2.getSupport(pos2.linear, pos2.angular, -d));

	if (dotProduct(d, simplex[1]) < 0.0f)
	{
		simplexPoints = 2;
		goto Distance_GJK;
	}

	d = normalAtOrigin(simplex[0], simplex[1]);

	while (++step <= GJK_STEPS_LIMIT)
	{
		// s0 s1 and origin lies on the same line
		if (d.isZero())
		{
			Vector2 s01 = simplex[1] - simplex[0];
			float dist = dotProduct(s01, -simplex[0]);

			if (dist < -GJK_EPSILON) // origin lies before s0
				d = -simplex[0];
			else if (dist - s01.lengthSq() > GJK_EPSILON) // origin lies after s1
				d = -simplex[1];
			else // origin lies on the segment
				d = normalVectorCCW(s01);
		}

		d = d.normalize();
		simplex[2] = (supports[2].first = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[2].second = shape2.getSupport(pos2.linear, pos2.angular, -d));


		if (dotProduct(d, simplex[2]) < GJK_EPSILON)
		{
			simplexPoints = 3;
			goto Distance_GJK;
		}

		if (dotProduct(d = normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < GJK_EPSILON)
		{
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else if (dotProduct(d = normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < GJK_EPSILON)
		{
			simplex[1] = simplex[2];
			supports[1] = supports[2];
		}
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
		supports[0] = supports[1];
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
			supports[1] = supports[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else
		{
			singlePointFlag = true;
			simplex[0] = simplex[2];
			supports[0] = supports[2];
			d = -simplex[0];
		}
	}

	step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		if (d.isZero())
		{
			distance = { {0.0f, 0.0f}, 0.0f };

			if (features)
			{
				if (!singlePointFlag)
				{
					if (supports[0].first != supports[1].first)
						features->f1.set(supports[0].first, supports[1].first);
					else
						features->f1.set(supports[0].first);

					if (supports[0].second != supports[1].second)
						features->f2.set(supports[0].second, supports[1].second);
					else
						features->f2.set(supports[0].second);
				}
				else
				{
					features->f1.set(supports[0].first);
					features->f2.set(supports[0].second);
				}
			}
			return;
		}

		d = d.normalize();
		simplex[2] = (supports[2].first = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[2].second = shape2.getSupport(pos2.linear, pos2.angular, -d));


		float dS2 = dotProduct(d, simplex[2]);
		float dS0 = dotProduct(d, simplex[0]);
		//if (dC - dA < std::abs(dC * EPSILON))
		if (dS2 - dS0 < GJK_EPSILON)
		{
			distance = { d, -dS0 };

			if (features)
			{
				features->f1 = shape1.getLocalClosestFeature(-pos1.angular * d);
				features->f1 = pos1.angular * features->f1;
				features->f1 = features->f1 + pos1.linear;

				features->f2 = shape2.getLocalClosestFeature(-pos2.angular * -d);
				features->f2 = pos2.angular * features->f2;
				features->f2 = features->f2 + pos2.linear;
			}

			return;
		}

		Vector2 s02 = simplex[2] - simplex[0];
		Vector2 s12 = simplex[2] - simplex[1];

		if (-dotProduct(s02, simplex[0]) < s02.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-s02.y, s02.x) * -crossProduct(s02, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[1] = simplex[2];
			supports[1] = supports[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else
		{
			singlePointFlag = true;
			d = -simplex[2];
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
	}

	assert(false && "GJK distance test passed max number of steps");

	distance = { {0.0f, 0.0f}, (float) INFINITY };
	return;

Depth_EPA:

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
	while (++step <= EPA_STEPS_LIMIT)
	{
		// get closest edge
		int edgeId = 0;
		for (int i = 1, iMax = (int)edges.size(); i < iMax; ++i)
			if (edges[i].magnitude < edges[edgeId].magnitude)
				edgeId = i;

		// calculate point
		Vector2 point = shape1.getSupport(pos1.linear, pos1.angular, edges[edgeId].direction) - shape2.getSupport(pos2.linear, pos2.angular, -edges[edgeId].direction);
		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < GJK_EPSILON)
		{
			distance.direction = edges[edgeId].direction;
			distance.magnitude = -newMagnitude;

			if (features)
			{
				features->f1 = shape1.getLocalClosestFeature(-pos1.angular * distance.direction);
				features->f1 = pos1.angular * features->f1;
				features->f1 = features->f1 + pos1.linear;

				features->f2 = shape2.getLocalClosestFeature(-pos2.angular * -distance.direction);
				features->f2 = pos2.angular * features->f2;
				features->f2 = features->f2 + pos2.linear;
			}

			return;
		}

		//unsigned int insIndex = (edgeId + 1) % polygon.size();
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

	distance = { {0.0f, 0.0f}, (float)-INFINITY };
	return;
};


bool PhysicsNarrowPhase::intersectGJK2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2) const
{
	Simplex simplex;

	Vector2 d = (shape2.getPosition(pos2.linear, pos2.angular) - shape1.getPosition(pos1.linear, pos1.angular)).normalize();
	simplex[0] = shape1.getSupport(pos1.linear, pos1.angular, d) - shape2.getSupport(pos2.linear, pos2.angular, -d);

	d = -simplex[0].normalize();
	simplex[1] = shape1.getSupport(pos1.linear, pos1.angular, d) - shape2.getSupport(pos2.linear, pos2.angular, -d);

	if (dotProduct(d, simplex[1]) < 0.0f)
		return false;

	d = normalAtOrigin(simplex[0], simplex[1]);

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		// s0 s1 and origin lies on the same line
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

		simplex[2] = shape1.getSupport(pos1.linear, pos1.angular, d) - shape2.getSupport(pos2.linear, pos2.angular, -d);
		if (dotProduct(d, simplex[2]) < EPSILON)
			return false;

		if (dotProduct(d = normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
			simplex[0] = simplex[2];
		else if (dotProduct(d = normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
			simplex[1] = simplex[2];
		else
			return true;
	}

	assert(false && "GJK test passed max number of steps");
	return false;
};


bool PhysicsNarrowPhase::getContactInformation(const Collider& c1, const Collider& c2, DirectionVector &distance, Vector2 &r1, Vector2 &r2, unsigned int &shapeIndex1, unsigned int &shapeIndex2)
{
	distance = { Vector2(0.0f), P_INF };
	shapeIndex1 = -1;
	shapeIndex2 = -1;
	Features features;

	for (unsigned int i = 0, iMax = c1.getShapeCount(); i < iMax; ++i)
	{
		for (unsigned int j = 0, jMax = c2.getShapeCount(); j < jMax; ++j)
		{
			// ADD MARGIN
			if (isOverlapping(c1.getShapeAABB(i), c2.getShapeAABB(j), COLLISION_BUFFER))
			{
				DirectionVector newDistance;
				Features newFeatures;
				{
					// CHANGE THIS TO DEDICATED FUNC!!!
					closestFeaturesGJK2(c1.getShape(i), c1.getOriginPosition(), c2.getShape(j), c2.getOriginPosition(), newDistance, &newFeatures);

					//move features to local spaces
					newFeatures.f1.segmentP1 = -(c1.getOriginPosition().angular) * (newFeatures.f1.segmentP1 - c1.getOriginPosition().linear);
					newFeatures.f1.segmentP2 = -(c1.getOriginPosition().angular) * (newFeatures.f1.segmentP2 - c1.getOriginPosition().linear);
					newFeatures.f2.segmentP1 = -(c2.getOriginPosition().angular) * (newFeatures.f2.segmentP1 - c2.getOriginPosition().linear);
					newFeatures.f2.segmentP2 = -(c2.getOriginPosition().angular) * (newFeatures.f2.segmentP2 - c2.getOriginPosition().linear);
				}

				if (distance.magnitude < newDistance.magnitude)
				{
					distance = newDistance;
					features = newFeatures;

					shapeIndex1 = i;
					shapeIndex2 = j;

				}
			}
		}
	}

	if (distance.magnitude > COLLISION_BUFFER)
		return false;


	Feature &f1 = features.f1, &f2 = features.f2;

	if (f1.type == Feature::FT_POINT && f2.type == Feature::FT_POINT)
	{
		r1 = f1.point;
		r2 = f2.point;
	}
	else if (f1.type == Feature::FT_POINT && f2.type == Feature::FT_SEGMENT)
	{
		r1 = f1.point;

		// p1 to world space
		r2 = c1.getOriginPosition().angular * (f1.point + c1.getOriginPosition().linear);
		// move by penetration depth
		r2 += (Vector2)distance;
		// to local space of c2
		r2 = -(c2.getOriginPosition().angular) * (r2 - c2.getOriginPosition().linear);
	}
	else if (f1.type == Feature::FT_SEGMENT && f2.type == Feature::FT_POINT)
	{
		r2 = f2.point;

		// p2 to world space
		r1 = c1.getOriginPosition().angular * (f2.point + c2.getOriginPosition().linear);
		// move by penetration depth
		r1 -= (Vector2)distance;
		// to local space of c1
		r1 = -(c1.getOriginPosition().angular) * (r1 - c1.getOriginPosition().linear);
	}
	else
	{
		Vector2 ctocNorm = normalVectorCCW(c2.getPosition().linear - c1.getPosition().linear).normalize();

		Vector2 ctocNormLoc1 = -c1.getPosition().angular * ctocNorm;
		float df1p1 = dotProduct(f1.segmentP1, ctocNormLoc1);
		float df1p2 = dotProduct(f1.segmentP2, ctocNormLoc1);
		if (df1p1 > df1p2)
		{
			std::swap(df1p1, df1p2);
			std::swap(f1.segmentP1, f1.segmentP2);
		}
		float df1 = df1p2 - df1p1;


		Vector2 ctocNormLoc2 = -c2.getPosition().angular * ctocNorm;

		float df2p1 = dotProduct(f2.segmentP1, ctocNormLoc2);
		float df2p2 = dotProduct(f2.segmentP2, ctocNormLoc2);
		if (df2p1 > df2p2)
		{
			std::swap(df2p1, df2p2);
			std::swap(f2.segmentP1, f2.segmentP2);
		}
		float df2 = df2p2 - df2p1;

		float dn = dotProduct((Vector2)distance, ctocNorm);
		float dn2 = dn / 2;

		// https://www.desmos.com/calculator/icsmf4kwij
		// left | x
		if (df1p2 < dn2)
		{
			// left | left
			if (df2p2 < -dn2)
			{
				r1 = f1.segmentP2 + ((f1.segmentP1 - f1.segmentP2) * (df1p2 - min(df1p2, df2p2 + dn)) / df1);
				r2 = f2.segmentP2 + ((f2.segmentP1 - f2.segmentP2) * (df2p2 - min(df1p2 - dn, df2p2)) / df2);
			}
			// left | middle
			else //if (df2p1 <= -dn2 && df2p2 >= -dn2)
			{
				r1 = f1.segmentP2;
				r2 = f2.segmentP2 + ((f2.segmentP1 - f2.segmentP2) * (df2p2 - df1p2 + dn) / df2);
			}
		}
		// right | x
		else if (df1p1 > dn2)
		{
			// right | right
			if (df2p1 > -dn2)
			{
				r1 = f1.segmentP2 + ((f1.segmentP1 - f1.segmentP2) * (df1p2 - max(df1p1, df2p1 + dn)) / df1);
				r2 = f2.segmentP2 + ((f2.segmentP1 - f2.segmentP2) * (df2p2 - max(df1p1 - dn, df2p1)) / df2);
			}
			// right | middle
			else //if (df2p1 <= -dn2 && df2p2 >= -dn2)
			{
				r1 = f1.segmentP1;
				r2 = f2.segmentP2 + ((f2.segmentP1 - f2.segmentP2) * (df2p2 - df1p1 + dn) / df2);
			}
		}
		// middle | x
		else //if (df1p1 <= dn2 && df1p2 >= dn2)
		{
			// middle | left
			if (df2p2 < -dn2)
			{
				r1 = f1.segmentP2 + ((f1.segmentP1 - f1.segmentP2) * (df1p2 - df2p2 - dn) / df1);
				r2 = f2.segmentP2;
			}
			// middle | right
			else if (df2p1 > -dn2)
			{
				r1 = f1.segmentP2 + ((f1.segmentP1 - f1.segmentP2) * (df1p2 - df2p1 - dn) / df1);
				r2 = f2.segmentP1;
			}
			// middle | middle
			else //if (df2p1 <= -dn2 && df2p2 >= -dn2)
			{
				r1 = f1.segmentP2 + ((f1.segmentP1 - f1.segmentP2) * (df1p2 - dn / 2) / df1);
				r2 = f2.segmentP2 + ((f2.segmentP1 - f2.segmentP2) * (df2p2 + dn / 2) / df2);
			}
		}
	}

	return true;
};


float PhysicsNarrowPhase::getRestitutionCoefficient(const MaterialID& material1, const MaterialID& material2)
{
	auto eIt = restitutionCoefficients.find(MaterialPair(material1, material2));
	if (eIt != restitutionCoefficients.end())
		return eIt->second;
	return DEFAULT_RESTITUTION_COEF;
};


void PhysicsNarrowPhase::calculateTentativeVelocityAndPosition(float timestep)
{
	unsigned int n = (unsigned int)colliders.size();

	std::vector<Force> Fext(n);
	computeExternalForces(Fext);

	// calc tentative velocity and position
	for (unsigned int i = 0; i < n; ++i)
	{
		if (colliders[i])
		{
			Velocity Vt = colliders[i]->getVelocity();

			if (colliders[i]->getMass() < (float)INFINITY) // only finite mass can have any change in velocity
			{
				Vt += (colliders[i]->getInvMassMx() * Fext[i]) * timestep;
				colliders[i]->setVelocity(Vt);
			}

			colliders[i]->setPosition(integrate(colliders[i]->getPosition(), Vt, timestep));
		}
	}

	//VMF[i] = colliders[i]->getVelocity() / timestep
};


float PhysicsNarrowPhase::distance(const ColliderShape &o1, const ColliderShape &o2)
{
	Vector2 A, B, C, d;

	d = o2.getWorldPosition() - o1.getWorldPosition();
	d = d.normalize();
	A = o1.getWorldSupport(d) - o2.getWorldSupport(-d);

	d = -A.normalize();
	B = o1.getWorldSupport(d) - o2.getWorldSupport(-d);

	Vector2 AB = B - A;
	float projDistSq = -dotProduct(AB, A); // dotProduct(segmentVect, 0 - A)

	if (projDistSq <= 0.0f)
		return A.length();

	if (projDistSq >= AB.lengthSq())
		return B.length();

	// A-B
	bool singlePointFlag = false;
	d = Vector2(-AB.y, AB.x) * -crossProduct(AB, A); // crossProduct(AB, 0 - A)

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		if (d.lengthSq() < EPSILON*EPSILON)
			return 0.0f;

		d = d.normalize();
		C = o1.getWorldSupport(d) - o2.getWorldSupport(-d);

		float dC = dotProduct(d, C);
		float dA = dotProduct(d, A);
		//if (dC - dA < std::abs(dC * EPSILON))
		if (dC - dA < EPSILON)
		{
			if (!singlePointFlag)
				return std::abs(dA);
			return A.length();
		}

		// simplex contains origin
		if (!singlePointFlag)
		{
			float crossAAB = crossProduct(A, B - A);
			if (crossAAB * crossProduct(B, C - B) >= 0.0f && crossAAB * crossProduct(C, A - C) >= 0.0f)
				return 0.0f;
		}

		Vector2 AC = C - A;
		Vector2 BC = C - B;

		if (-dotProduct(AC, A) < AC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-AC.y, AC.x) * -crossProduct(AC, A); // crossProduct(AB, 0 - A)
			B = C;
		}
		else if (-dotProduct(BC, B) < BC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(BC.y, -BC.x) * crossProduct(BC, A); // crossProduct(AB, 0 - A)
			A = C;
		}
		else
		{
			singlePointFlag = true;
			d = -C;
			A = C;
		}
	}

	return 0.0f;
};

bool PhysicsNarrowPhase::distanceDirection(const ColliderShape &shape1, const Vector2 &position1, const Quaternion &rotation1, const ColliderShape &shape2, const Vector2 &position2, const Quaternion &rotation2, DirectionVector &distance)
{
	Vector2 A, B, C, d;

	d = shape2.getPosition(position2, rotation2) - shape1.getPosition(position1, rotation1);
	d = d.normalize();
	A = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);

	d = -A.normalize();
	B = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);

	Vector2 AB = B - A;
	float projDistSq = -dotProduct(AB, A); // dotProduct(segmentVect, 0 - A)

	if (projDistSq <= 0.0f)
		return A.length();

	if (projDistSq >= AB.lengthSq())
		return B.length();

	// A-B
	bool singlePointFlag = false;
	d = Vector2(-AB.y, AB.x) * -crossProduct(AB, A); // crossProduct(AB, 0 - A)

	unsigned int step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		if (d.lengthSq() < EPSILON*EPSILON)
			return 0.0f;

		d = d.normalize();
		C = shape1.getSupport(position1, rotation1, d) - shape2.getSupport(position2, rotation2, -d);

		float dC = dotProduct(d, C);
		float dA = dotProduct(d, A);
		//if (dC - dA < std::abs(dC * EPSILON))
		if (dC - dA < EPSILON)
		{
			distance.magnitude = dC;
			distance.direction = d;
			return true;
		}

		// simplex contains origin
		if (!singlePointFlag)
		{
			float crossAAB = crossProduct(A, B - A);
			if (crossAAB * crossProduct(B, C - B) >= 0.0f && crossAAB * crossProduct(C, A - C) >= 0.0f)
				return 0.0f;
		}

		Vector2 AC = C - A;
		Vector2 BC = C - B;

		if (-dotProduct(AC, A) < AC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-AC.y, AC.x) * -crossProduct(AC, A); // crossProduct(AB, 0 - A)
			B = C;
		}
		else if (-dotProduct(BC, B) < BC.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(BC.y, -BC.x) * crossProduct(BC, A); // crossProduct(AB, 0 - A)
			A = C;
		}
		else
		{
			singlePointFlag = true;
			d = -C;
			A = C;
		}
	}

	return false;
};


bool PhysicsNarrowPhase::closestPoints(const ColliderShape &o1, const ColliderShape &o2, Vector2 &point1, Vector2 &point2)
{
	Vector2 supA1, supA2, supB1, supB2, supC1, supC2;
	Vector2 A, B, C, d;

	d = o2.getWorldPosition() - o1.getWorldPosition();
	d = d.normalize();
	A = (supA1 = o1.getWorldSupport(d)) - (supA2 = o2.getWorldSupport(-d));

	d = -A.normalize();
	B = (supB1 = o1.getWorldSupport(d)) - (supB2 = o2.getWorldSupport(-d));

	Vector2 AB = B - A;
	float projDistSq = -dotProduct(AB, A); // dotProduct(segmentVect, 0 - A)

	if (projDistSq <= 0.0f)
	{
		point1 = supA1;
		point2 = supA2;
		return true;
	}

	if (projDistSq >= AB.lengthSq())
	{
		point1 = supB1;
		point2 = supB2;
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
		C = (supC1 = o1.getWorldSupport(d)) - (supC2 = o2.getWorldSupport(-d));

		float dC = dotProduct(d, C);
		// d*c >= d*a // keep d normalized, measure against relative epsilon to stop false negatives
		// C didnt get closer than previous point
		// point is on the line or A if singlePointFlag
		//if (dC - dotProduct(d, A) < std::abs(dC * EPSILON))
		if (dC - dotProduct(d, A) < EPSILON)
		{
			if (!singlePointFlag)
			{
				Vector2 l = B - A;
				float lambda2 = -dotProduct(A, l) / dotProduct(l, l);
				float lambda1 = 1.0f - lambda2;

				point1 = supA1 * lambda1 + supB1 * lambda2;
				point2 = supA2 * lambda1 + supB2 * lambda2;
			}
			else
			{
				point1 = supA1;
				point2 = supA2;
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



bool PhysicsNarrowPhase::closestPoints(const Collider &o1, const Collider &o2, Vector2 &point1, Vector2 &point2)
{
	Vector2 tmpP1, tmpP2;
	float currentDistSq = (float)INFINITY;

	for (unsigned int i = 0, o1sMax = o1.getShapeCount(); i < o1sMax; ++i)
	{
		for (unsigned int j = 0, o2sMax = o2.getShapeCount(); j < o2sMax; ++j)
		{
			bool test = closestPoints(o1.getShape(i), o2.getShape(i), tmpP1, tmpP2);
			if (!test)
				return false;

			float tmpDistSq = (tmpP2 - tmpP1).lengthSq();
			if (currentDistSq > tmpDistSq)
			{
				currentDistSq = tmpDistSq;
				point1 = tmpP1;
				point2 = tmpP2;
			}
		}
	}
	return true;
};


void PhysicsNarrowPhase::intersectionDepth2(const ColliderShape &shape1, const Position &pos1, const ColliderShape &shape2, const Position &pos2, DirectionVector &depth, Features &closestFeatures, bool getDistance)
{
	std::vector<Vector2> simplex(3);
	std::vector<SupportPair> supports(3);
	unsigned int simplexPoints;
	unsigned int step = 0;

	Vector2 d = (shape2.getPosition(pos2.linear, pos2.angular) - shape1.getPosition(pos1.linear, pos1.angular)).normalize();
	simplex[0] = (supports[0].A = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[0].B = shape2.getSupport(pos2.linear, pos2.angular, -d));

	d = -simplex[0].normalize();
	simplex[1] = (supports[1].A = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[1].B = shape2.getSupport(pos2.linear, pos2.angular, -d));

	if (dotProduct(d, simplex[1]) < 0.0f)
	{
		if (getDistance)
		{
			simplexPoints = 2;
			goto Distance_GJK;
		}
		depth.magnitude = -INFINITY;
		return;
	}

	while (++step <= GJK_STEPS_LIMIT)
	{
		d = normalAtOrigin(simplex[0], simplex[1]);
		if (d.isZero())
			d = normalVectorCCW(simplex[1]);
		d = d.normalize();

		simplex[2] = (supports[2].A = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[2].B = shape2.getSupport(pos2.linear, pos2.angular, -d));
		if (dotProduct(d, simplex[2]) < 0.0f)
		{
			if (getDistance)
			{
				simplexPoints = 3;
				goto Distance_GJK;
			}
			depth.magnitude = -INFINITY;
			return;
		}

		if (dotProduct(normalAtOrigin(simplex[1], simplex[2]), simplex[0]) < EPSILON)
		{
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else if (dotProduct(normalAtOrigin(simplex[2], simplex[0]), simplex[1]) < EPSILON)
		{
			simplex[1] = simplex[2];
			supports[1] = supports[2];
		}
		else
			goto Depth_EPA;
	}

	assert(step <= GJK_STEPS_LIMIT && "GJK test passed max number of steps");


Distance_GJK:

	bool singlePointFlag;

	if (simplexPoints == 2)
	{
		singlePointFlag = true;
		simplex[0] = simplex[1];
		supports[0] = supports[1];
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
			supports[1] = supports[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else
		{
			singlePointFlag = true;
			simplex[0] = simplex[2];
			supports[0] = supports[2];
			d = -simplex[0];
		}
	}

	step = 0;
	while (++step <= GJK_STEPS_LIMIT)
	{
		if (d.lengthSq() < EPSILON*EPSILON)
		{
			depth = { {0.0f, 0.0f}, 0.0f };
			break;
		}

		d = d.normalize();
		simplex[2] = (supports[2].A = shape1.getSupport(pos1.linear, pos1.angular, d)) - (supports[2].B = shape2.getSupport(pos2.linear, pos2.angular, -d));

		float dS2 = dotProduct(d, simplex[2]);
		float dS0 = dotProduct(d, simplex[0]);
		if (dS2 - dS0 < EPSILON)
		{
			depth = { d, -dS0 };
			break;
		}

		Vector2 s02 = simplex[2] - simplex[0];
		Vector2 s12 = simplex[2] - simplex[1];

		if (-dotProduct(s02, simplex[0]) < s02.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(-s02.y, s02.x) * -crossProduct(s02, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[1] = simplex[2];
			supports[1] = supports[2];
		}
		else if (-dotProduct(s12, simplex[1]) < s12.lengthSq())
		{
			singlePointFlag = false;
			d = Vector2(s12.y, -s12.x) * crossProduct(s12, simplex[0]); // crossProduct(AB, 0 - A)
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
		else
		{
			singlePointFlag = true;
			d = -simplex[2];
			simplex[0] = simplex[2];
			supports[0] = supports[2];
		}
	}

	assert(step <= GJK_STEPS_LIMIT && "GJK distance test passed max number of steps");

	// fill features if required
	if (!singlePointFlag)
	{
		if (supports[0].A != supports[1].A)
			closestFeatures.f1.set(supports[0].A, supports[1].A);
		else
			closestFeatures.f1.set(supports[0].A);

		if (supports[0].B != supports[1].B)
			closestFeatures.f2.set(supports[0].B, supports[1].B);
		else
			closestFeatures.f2.set(supports[0].B);
	}
	else
	{
		closestFeatures.f1.set(supports[0].A);
		closestFeatures.f2.set(supports[0].B);
	}

	return;


Depth_EPA:

	std::vector<Vector2> &polygon = simplex;
	std::vector<DirectionVector> edges(3);
	int edgeCount = 3;

	// simplex is CCW so swap 2 last point
	if (dotProduct(normalVectorCW(simplex[1] - simplex[0]), simplex[2] - simplex[0]) < 0.0f)
	{
		std::swap(polygon[1], polygon[2]);
		std::swap(supports[1], supports[2]);
	}

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
		SupportPair support = { shape1.getSupport(pos1.linear, pos1.angular, edges[edgeId].direction), shape2.getSupport(pos2.linear, pos2.angular, -edges[edgeId].direction) };
		Vector2 point = support.A - support.B;

		float newMagnitude = dotProduct(point, edges[edgeId].direction);
		if (newMagnitude - edges[edgeId].magnitude < EPA_TOLERANCE)
		{
			depth.direction = edges[edgeId].direction;
			depth.magnitude = newMagnitude;

			SupportPair &s1 = supports[edgeId];
			SupportPair &s2 = supports[(edgeId + 1) % supports.size()];

			if (s1.A != s2.A)
				closestFeatures.f1.set(s1.A, s2.A);
			else
				closestFeatures.f1.set(s1.A);

			if (s1.B != s2.B)
				closestFeatures.f2.set(s1.B, s2.B);
			else
				closestFeatures.f2.set(s1.B);

			return;
		}


		unsigned int insIndex = edgeId + 1;
		// insert point
		polygon.insert(polygon.begin() + insIndex, point);

		// insert supports
		supports.insert(supports.begin() + insIndex, support);

		// split edge and calc distance on new edges
		edges.insert(edges.begin() + edgeId + 1, 1, {});

		++edgeCount;

		edges[edgeId].direction = normalVectorCCW(polygon[edgeId + 1] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);

		++edgeId;
		edges[edgeId].direction = normalVectorCCW(polygon[(edgeId + 1) % edgeCount] - polygon[edgeId]).normalize();
		edges[edgeId].magnitude = dotProduct(edges[edgeId].direction, polygon[edgeId]);
	}

	assert(step <= EPA_STEPS_LIMIT && "EPA test passed max number of steps");

	return;
};



// deepest point solver TOI_DISTANCE_EPSILON <= dist <= 2 x TOI_DISTANCE_EPSILON
// deepest point distance undershoot by TOI_DISTANCE_EPSILON to mitigate differences between GJK results caused by floating points errors 

float PhysicsNarrowPhase::deepestPointSolverPtoP(const Collider &collider1, const ColliderShape &shape1, const Position& position1, const Collider &collider2, const ColliderShape &shape2, const Position& position2, DirectionVector &distance, Features &features, float timestep, float extTn)
{
	float t1 = extTn, t2 = 1.0f, time, tn, s1, s2, sn;
	Position pos1 = position1, pos2 = position2;

	Vector2 p1, p2;
	Vector2 &u = distance.direction;

	// don't need separate pos/rot after getting deepest point
	{
		Position pos1t2, pos2t2;
		pos1t2 = collider1.getPartialPositionAndRotation(time = timestep * (t2 - 1.0f));
		pos2t2 = collider2.getPartialPositionAndRotation(time);

		// deepest points in local space
		p1 = shape1.getLocalSupport(-pos1t2.angular * u);
		p2 = shape2.getLocalSupport(-pos2t2.angular * -u);

		// get distance projected on u on t1 and t2
		s2 = dotProduct(u, (pos2t2.linear + pos2t2.angular * p2) - (pos1t2.linear + pos1t2.angular * p1));
	}
	s1 = dotProduct(u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1));

	// looking for root
	unsigned int step = 0;
	while (++step <= TOI_FEATURE_STEPS_LIMIT)
	{
		// Bisection Method  |  False Position Method
		tn = t1;
		tn += (step & 1) ? ((t2 - t1) * 0.5f) : ((s1 * (t2 - t1)) / (s1 - s2));
		// False Position Method  |  Bisection Method
		//tn = t1 + (step & 1) ? ((s1 * (t2 - t1)) / (s1 - s2)) : ((t2 - t1) * 0.5f);

		// get tn positions and rotations
		pos1 = collider1.getPartialPositionAndRotation(time = timestep * (tn - 1.0f));
		pos2 = collider2.getPartialPositionAndRotation(time);

		// get distance projected on u
		sn = dotProduct(u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;
		if (sn > TOI_DISTANCE_EPSILON)
		{
			t1 = tn;
			s1 = sn;
		}
		else if (sn < 0.0f)
		{
			t2 = tn;
			s2 = sn;
		}
		else
		{
			t2 = tn;

			// get support points on root time
			p1 = shape1.getLocalSupport(-pos1.angular * u);
			p2 = shape2.getLocalSupport(-pos2.angular * -u);

			// check if there are no deeper points
			s2 = dotProduct(u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;

			// only break if there are no deeper points
			if (s2 >= 0.0f)
				return t2;

			float t10 = t1 * 0.1f;
			do
			{
				t1 -= t10;
				pos1 = collider1.getPartialPositionAndRotation(time = timestep * (t1 - 1.0f));
				pos2 = collider2.getPartialPositionAndRotation(time);
				Vector2 pp2 = pos2.linear + pos2.angular * p2;
				Vector2 pp1 = pos1.linear + pos1.angular * p1;
				Vector2 pp21 = pp2 - pp1;
				float s11 = dotProduct(u, pp21);
				s1 = dotProduct(u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;
			} while (s1 < 0.0);
		}
	}

	//assert(step <= TOI_FEATURE_STEPS_LIMIT && "point to point root solver exceded number of steps");
	return t1; // safe choice
};

float PhysicsNarrowPhase::deepestPointSolverStoP(const Collider &collider1, const ColliderShape &shape1, const Position& position1, const Collider &collider2, const ColliderShape &shape2, const Position& position2, DirectionVector &distance, Features &features, float timestep, float extTn)
{
	float t1 = extTn, t2 = 1.0f, time, tn, s1, s2, sn;
	Position pos1 = position1, pos2 = position2;

	// 
	Vector2 u = -pos1.angular * distance.direction;
	Vector2 p1 = -pos1.angular * (features.f1.segmentP1 - position1.linear), p2;

	// don't need separate pos/rot after getting deepest point
	{
		Position pos1t2, pos2t2;
		pos1t2 = collider1.getPartialPositionAndRotation(time = timestep * (t2 - 1.0f));
		pos2t2 = collider2.getPartialPositionAndRotation(time);

		// deepest points in local space
		p2 = shape2.getLocalSupport(-pos2t2.angular * pos1t2.angular * -u);

		// get distance projected on u on t1 and t2
		s2 = dotProduct(pos1t2.angular * u, (pos2t2.linear + pos2t2.angular * p2) - (pos1t2.linear + pos1t2.angular * p1));
	}
	s1 = dotProduct(pos1.angular * u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1));

	// looking for root
	unsigned int step = 0;
	while (++step <= TOI_FEATURE_STEPS_LIMIT)
	{
		// Bisection Method  |  False Position Method
		tn = t1;
		tn += (step & 1) ? ((t2 - t1) * 0.5f) : ((s1 * (t2 - t1)) / (s1 - s2));
		// False Position Method  |  Bisection Method
		//tn = t1 + (step & 1) ? ((s1 * (t2 - t1)) / (s1 - s2)) : ((t2 - t1) * 0.5f);

		// get tn positions and rotations
		pos1 = collider1.getPartialPositionAndRotation(time = timestep * (tn - 1.0f));
		pos2 = collider2.getPartialPositionAndRotation(time);

		// get distance projected on u
		sn = dotProduct(pos1.angular * u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;
		if (sn > TOI_DISTANCE_EPSILON)
		{
			t1 = tn;
			s1 = sn;
		}
		else if (sn < 0.0f)
		{
			t2 = tn;
			s2 = sn;
		}
		else
		{
			t2 = tn;

			// get support points on root time
			p2 = shape2.getLocalSupport(-pos2.angular * pos1.angular * -u);

			// check if there are no deeper points
			s2 = dotProduct(pos1.angular * u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;

			// only break if there are no deeper points
			if (s2 >= 0.0f)
				return t2;

			float t10 = t1 * 0.1f;
			do
			{
				t1 -= t10;
				pos1 = collider1.getPartialPositionAndRotation(time = timestep * (t1 - 1.0f));
				pos2 = collider2.getPartialPositionAndRotation(time);
				s1 = dotProduct(pos1.angular * u, (pos2.linear + pos2.angular * p2) - (pos1.linear + pos1.angular * p1)) - TOI_DISTANCE_EPSILON;
			} while (s1 < 0.0);
		}
	}

	//assert(step <= TOI_FEATURE_STEPS_LIMIT && "point to point root solver exceded number of steps");
	return t1; // safe choice
};


void PhysicsNarrowPhase::deepestPointSolver(const Collider &collider1, unsigned int shapeIndex1, const Collider &collider2, unsigned int shapeIndex2, float timestep, float &tn, Features &features)
{
	const ColliderShape &shape1 = collider1.getShape(shapeIndex1);
	const ColliderShape &shape2 = collider2.getShape(shapeIndex2);

	float time, t1 = tn, t2 = 1.0f, tRoot;
	
	Position pos1, pos2;

	DirectionVector d;
	Features &f = features;

	unsigned int step = 0;
	while (++step <= DEEPEST_POINT_SOLVER_STEPS_LIMIT)
	{
		// get current position/rotation on t1
		pos1 = collider1.getPartialPositionAndRotation(time = timestep * (t1 - 1.0f));
		pos2 = collider2.getPartialPositionAndRotation(time);

		// distant should be positive in nearly all cases, rarely negative close to 0 (below COLLISION_DISTANCE_EPSILON)
		closestFeaturesGJK2(shape1, pos1, shape2, pos2, d, &f);

		// check for TOI
		if (d.magnitude <= COLLISION_DISTANCE_EPSILON)
		{
			tn = t1;
			return; // fount TOI
		}

		// call deepest point resolver based on features
		FeatureHash featuresType = FEATURE_HASH(f.f1.type, f.f2.type);
		switch (featuresType)
		{
		case FEATURE_HASH(Feature::FT_POINT, Feature::FT_POINT):
		{
			tRoot = deepestPointSolverPtoP(collider1, shape1, pos1, collider2, shape2, pos2, d, f, timestep, t1);
			break;
		}
		case FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_SEGMENT): // treat as segment - point
			f.f2.type = Feature::FT_POINT;

		case FEATURE_HASH(Feature::FT_SEGMENT, Feature::FT_POINT):
		{
			tRoot = deepestPointSolverStoP(collider1, shape1, pos1, collider2, shape2, pos2, d, f, timestep, t1);
			break;
		}
		case FEATURE_HASH(Feature::FT_POINT, Feature::FT_SEGMENT):
		{
			f = { {f.f2.segmentP1, f.f2.segmentP2}, {f.f1.point} }; // swap features
			d = -d;
			tRoot = deepestPointSolverStoP(collider2, shape2, pos2, collider1, shape1, pos1, d, f, timestep, t1);
			break;
		}
		default:
			assert(false && "undefined feature set");
		}

		t1 = tRoot;
	}

	//assert(step <= DEEPEST_POINT_SOLVER_STEPS_LIMIT && "deepest point solver exceded number of steps");
	// safe choice
	tn = t1;
};


void PhysicsNarrowPhase::invalidateCollider(const Collider *_collider)
{
	broadPhase.invalidateColliderNode(_collider->getPhysicsBroadPhaseId());
};


void PhysicsNarrowPhase::setBroadphaseMargin(float _boxMargin)
{
	broadPhase.setMargin(_boxMargin);
};


int32_t PhysicsNarrowPhase::addColider(ColliderSPtr _collider)
{
	assert(_collider && "nullptr");

	int32_t colliderId;
	if (collidersFreeId.size())
	{
		colliderId = collidersFreeId.front();
		collidersFreeId.pop_front();
		colliders[colliderId] = _collider;
	}
	else
	{
		colliderId = (int32_t)colliders.size();
		colliders.emplace_back(_collider);
	}

	collidersMap.insert({ _collider->getName(), colliderId });
	_collider->setPhysicsNarrowPhaseId(colliderId);
	broadPhase.addCollider(*_collider);

	return colliderId;
};


void PhysicsNarrowPhase::removeCollider(int32_t _id)
{
	assert(_id >= 0 && _id < colliders.size() && "Trying to remove unregistered collider");

	broadPhase.removeCollider(*colliders[_id]);
	colliders[_id]->setPhysicsNarrowPhaseId(INVALID_INDEX);

	collidersMap.erase(colliders[_id]->getName());

	if (_id < colliders.size() - 1)
	{
		colliders[_id].reset();
		collidersFreeId.push_back(_id);
	}
	else
	{
		colliders.pop_back();
	}
};


void PhysicsNarrowPhase::removeCollider(const char _colliderId[COLLIDER_ID_LEN])
{
	auto it = collidersMap.find(_colliderId);
	assert(it != collidersMap.end() && "Trying to remove unregistered collider");
	removeCollider(it->second);
};


void PhysicsNarrowPhase::addMaterialRestitutionCoefficient(MaterialID mId1, MaterialID mId2, float e)
{
	MaterialPair mp = { mId1, mId2 };
	restitutionCoefficients.emplace(mp, e);
};


void PhysicsNarrowPhase::registerAffector(ColliderAffectorSPtr affector, AFFECTOR_SCOPE scope)
{
	assert(affectors.find(affector->getName()) == affectors.end() && "affector name already registered");
	AffectorStruct as = { affector, scope };
	affectors.insert({ affector->getName(), as });
};


void PhysicsNarrowPhase::unregisterAffector(const char affectorName[AFFECTOR_ID_LEN])
{
	assert(affectors.find(affectorName) != affectors.end() && "affector name not registered");
	affectors.erase(affectorName);
};


ColliderAffectorSPtr PhysicsNarrowPhase::getAffector(const char affectorName[AFFECTOR_ID_LEN]) const
{
	auto it = affectors.find(affectorName);
	assert(it != affectors.end() && "affector name not registered");
	return it->second.affectorPtr;
};


void PhysicsNarrowPhase::registerAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], int32_t colliderId)
{
	auto it = affectors.find(affectorName);
	assert(it != affectors.end() && "affector name not registered");

	switch (it->second.scope)
	{
	case AS_ONE:
		assert(it->second.colliderList.size() == 0 && "affector already have connected collider");
	case AS_LIST:
		it->second.colliderList.push_back(colliderId);
		break;
	}
};


void PhysicsNarrowPhase::registerAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], const char colliderName[COLLIDER_ID_LEN])
{
	auto it = collidersMap.find(colliderName);
	assert(it != collidersMap.end() && "collider not registered");
	registerAffectedCollider(affectorName, it->second);
};


void PhysicsNarrowPhase::unregisterAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], int32_t colliderId)
{
	auto it = affectors.find(affectorName);
	assert(it != affectors.end() && "affector name not registered");
	assert(it->second.scope != AS_ALL && "cannot unregister collider from affector with scope all");

	auto& list = it->second.colliderList;
	auto clIt = std::find(list.begin(), list.end(), colliderId);
	if (clIt != list.end())
		list.erase(clIt);
};


void PhysicsNarrowPhase::unregisterAffectedCollider(const char affectorName[AFFECTOR_ID_LEN], const char colliderName[COLLIDER_ID_LEN])
{
	auto it = collidersMap.find(colliderName);
	assert(it != collidersMap.end() && "collider not registered");
	unregisterAffectedCollider(affectorName, it->second);
};


void PhysicsNarrowPhase::addConstraint(ConstraintSPtr constraint, int32_t colliderId1, int32_t colliderId2)
{
	assert(colliderId1 >= 0 && colliderId1 < colliders.size() && colliderId2 >= 0 && colliderId2 < colliders.size() && "collider not registered");
	constraints.emplace_back(colliderId1, colliderId2, constraint, 0.0f);
};


void PhysicsNarrowPhase::addConstraint(ConstraintSPtr constraint, const char colliderName1[COLLIDER_ID_LEN], const char colliderName2[COLLIDER_ID_LEN])
{
	auto it1 = collidersMap.find(colliderName1);
	auto it2 = collidersMap.find(colliderName2);

	assert(it1 != collidersMap.end() && it2 != collidersMap.end() && "collider not registered");

	addConstraint(constraint, it1->second, it2->second);
};


void PhysicsNarrowPhase::removeConstraint(const char _name[CONSTRAINT_ID_LEN])
{
	auto it = std::find_if(constraints.begin(), constraints.end(), [_name](ConstraintData cd) {
		return strncmp(_name, cd.C->getName(), CONSTRAINT_ID_LEN) == 0;
	});

	assert(it != constraints.end() && "constraint not registered");

	std::swap(*it, constraints.back());
	constraints.pop_back();
};


void PhysicsNarrowPhase::removeConstraint(ConstraintSPtr constraint)
{
	removeConstraint(constraint->getName());
};


void PhysicsNarrowPhase::update(const float timestep)
{
	queryCollisions(timestep);

	computeVelocityConstraints(timestep);
};


void PhysicsNarrowPhase::computeExternalForces(std::vector<Force>& Fext)
{
	assert(Fext.size() == colliders.size() && "incorrect array size");

	for (auto it = affectors.begin(), itEnd = affectors.end(); it != itEnd; ++it)
	{
		const ColliderAffector* affector = it->second.affectorPtr.get();
		if (it->second.scope == AS_ALL)
		{
			for (unsigned int i = 0, iMax = (unsigned int)colliders.size(); i < iMax; ++i)
				Fext[i] += affector->generateForce(*colliders[i].get());
		}
		else // AS_LIST || AS_ONE
		{
			const auto& list = it->second.colliderList;
			for (unsigned int i = 0, iMax = (unsigned int)list.size(); i < iMax; ++i)
				Fext[i] += affector->generateForce(*colliders[list[i]].get());
		}
	}
};


void PhysicsNarrowPhase::computeVelocityConstraints(float timestep)
{
	unsigned int s = (unsigned int)constraints.size();
	unsigned int n = (unsigned int)colliders.size();

	float timeRec = 1.0f / timestep;


	std::vector<Force> Fext(n);
	computeExternalForces(Fext);

	std::vector<Velocity> Vt(n);
	std::vector<Position> Xt(n);
	// dt^-1 * V1 + M^-1 * Fext
	std::vector<Velocity> VMF(n);


	// calc tentative velocity and position
	for (unsigned int i = 0; i < n; ++i)
	{
		if (colliders[i]->getMass() < (float)INFINITY)
		{
			const Velocity& v1 = colliders[i]->getVelocity();
			Velocity vd = colliders[i]->getInvMassMx() * Fext[i];
			VMF[i] = v1 * timeRec + vd;
			//MF[i] = colliders[i]->getInvMassMx * forces[i];
			//VMF[i] = v1 * timeRec + MF[i];

			Vt[i] = v1 + vd * timestep;
			//Vt[i] = v1 + MF[i] * timestep;
		}
		else // infinite mass, no velocity change possible
		{
			Vt[i] = colliders[i]->getVelocity();
			VMF[i] = Vt[i] * timeRec;
		}

		const Position& x1 = colliders[i]->getPosition();
		Xt[i] = integrate(x1, Vt[i], timestep);
	}


	std::vector<IndexedJacobian> J(s);

	// B = M^-1 * J^T
	// B structure in 2D is the same as J^T
	std::vector<IndexedJacobian> B(s);
	std::vector<float> eta(s);

	// calc position constraints, Jacobians and eta
	for (unsigned int i = 0; i < s; ++i)
	{
		int32_t& p1 = constraints[i].b1;
		int32_t& p2 = constraints[i].b2;

		ConstraintValue cv = constraints[i].C->calculate(timestep, Vt[p1], Xt[p1], Vt[p2], Xt[p2]);
		J[i] = { p1, p2, cv.J1, cv.J2 };
		B[i] = { p1, p2, colliders[p1]->getInvMassMx() * cv.J1, colliders[p2]->getInvMassMx() * cv.J2 };

		float zeta = -constraints[i].C->getCoefficient() * cv.C;
		eta[i] = timeRec * zeta - (J[i].J1 * VMF[J[i].b1] + J[i].J2 * VMF[J[i].b2]);
	}



	std::vector<Velocity> a(n);
	std::vector<float> d(s);

	// Projected Gauss-Seidel

	// B * lambda , B is sparse
	for (unsigned int i = 0; i < s; ++i)
	{
		a[B[i].b1] += B[i].J1 * constraints[i].lambda;
		a[B[i].b2] += B[i].J2 * constraints[i].lambda;

		// change to reciprocal to limit div in next loop
		d[i] = J[i].J1 * B[i].J1 + J[i].J2 * B[i].J2;
	}

	float maxLambdaChange;
	unsigned int steps = 0;
	while (steps++ < 100)
	{
		maxLambdaChange = 0.0f;

		for (unsigned int i = 0; i < s; ++i)
		{
			int32_t& b1 = J[i].b1;
			int32_t& b2 = J[i].b2;

			float deltaLambda = (eta[i] - J[i].J1 * a[b1] - J[i].J2 * a[b2]) / d[i];

			float lambda0i = constraints[i].lambda;

			constraints[i].lambda = constraints[i].C->clampLambda(lambda0i + deltaLambda);
			deltaLambda = constraints[i].lambda - lambda0i;

			a[b1] += B[i].J1 * deltaLambda;
			a[b2] += B[i].J2 * deltaLambda;

			maxLambdaChange = max(maxLambdaChange, deltaLambda);
		}

		if (maxLambdaChange < PGS_LAMBDA_MAX_DELTA)
			break;
	}


	for (unsigned int i = 0; i < s; ++i)
	{
		// Vc = dt * B(n-s) * lambda(n-1)
		Vt[B[i].b1] += B[i].J1 * (constraints[i].lambda * timestep);
		Vt[B[i].b2] += B[i].J2 * (constraints[i].lambda * timestep);
	}


	// integrate constraint forces
	for (unsigned int i = 0; i < n; ++i)
	{
		colliders[i]->setVelocity(Vt[i]);
		colliders[i]->setPosition(integrate(colliders[i]->getPosition(), Vt[i], timestep));
	}
};


bool PhysicsNarrowPhase::intersect(const Collider &collider, bool broadphaseOnly) const
{
	ColliderList cList = broadPhase.query(collider.getAABB());

	if (!cList.size()) return false;
	if (broadphaseOnly) return true;

	const Position& pos1 = collider.getPosition();

	for (unsigned int ic = 0, icMax = (unsigned int)cList.size(); ic < icMax; ++ic)
	{
		const Position& pos2 = cList[ic]->getPosition();

		for (unsigned int is2 = 0, is2Max = (unsigned int)cList[ic]->getShapeCount(); is2 < is2Max; ++is2)
		{
			const ColliderShape &shape2 = cList[ic]->getShape(is2);
			const AABB& shape2aabb = cList[ic]->getShapeAABB(is2);

			for (unsigned int is1 = 0, is1Max = (unsigned int)collider.getShapeCount(); is1 < is1Max; ++is1)
			{
				if (shape2aabb.isOverlapping(collider.getShapeAABB(is1)) && intersectGJK2(collider.getShape(is1), pos1, shape2, pos2))
					return true;
			}
		}
	}

	return false;
};


bool PhysicsNarrowPhase::continuosCollisionCheck(Collider& collider1, Collider& collider2, const float timestep, CollisionData& collisionData)
{
	Position pos1 = collider1.getPosition(), pos2 = collider2.getPosition();

	float distance = (float)INFINITY;
	unsigned short shapeIndex1;
	unsigned short shapeIndex2;

	for (unsigned int csi1 = 0, csMax1 = collider1.getShapeCount(); csi1 < csMax1; ++csi1)
	{
		for (unsigned int csi2 = 0, csMax2 = collider2.getShapeCount(); csi2 < csMax2; ++csi2)
		{
			// test shapes AABB, those shoud be updated by broadphase already
			if (!isOverlapping(collider1.getShapeAABB(csi1), collider2.getShapeAABB(csi2)))
				continue;

			DirectionVector tmpDepth;
			closestFeaturesGJK2(collider1.getShape(csi1), pos1, collider2.getShape(csi2), pos2, tmpDepth);
			if (tmpDepth.magnitude < -COLLISION_DISTANCE_EPSILON && tmpDepth.magnitude < distance)
			//if (tmpDepth.magnitude <= COLLISION_DISTANCE_EPSILON && tmpDepth.magnitude < distance)
			{
				distance = tmpDepth.magnitude;
				shapeIndex1 = csi1;
				shapeIndex2 = csi2;
			}
		}
	}

	// no collision
	if (distance >= -COLLISION_DISTANCE_EPSILON)
	//if (distance > COLLISION_DISTANCE_EPSILON)
		return false;

	// exact collision distance means collision at end of frame so frameTOI = 0
	float frameTOI = 0.0f;
	float TOI = 0.0f;
	Features f;
	unsigned short tmpShapeIndex1 = shapeIndex1;
	unsigned short tmpShapeIndex2 = shapeIndex2;

	// calculate TOI
	while (distance < -COLLISION_DISTANCE_EPSILON)
	{
		deepestPointSolver(collider1, shapeIndex1 = tmpShapeIndex1, collider2, shapeIndex2 = tmpShapeIndex2, timestep, TOI, f);

		frameTOI = timestep * (TOI - 1.0f);
		pos1 = collider1.getPartialPositionAndRotation(frameTOI);
		pos2 = collider2.getPartialPositionAndRotation(frameTOI);

		distance = (float)INFINITY;
		for (unsigned int csi1 = 0, csMax1 = collider1.getShapeCount(); csi1 < csMax1; ++csi1)
		{
			for (unsigned int csi2 = 0, csMax2 = collider2.getShapeCount(); csi2 < csMax2; ++csi2)
			{
				const ColliderShape &shape1 = collider1.getShape(csi1);
				const ColliderShape &shape2 = collider2.getShape(csi2);

				if (!isOverlapping(shape1.getAABB(pos1.linear, pos1.angular), shape2.getAABB(pos2.linear, pos2.angular)))
					continue;

				DirectionVector tmpDepth;
				closestFeaturesGJK2(shape1, pos1, shape2, pos2, tmpDepth);
				if (tmpDepth.magnitude < distance)
				{
					distance = tmpDepth.magnitude;
					tmpShapeIndex1 = csi1;
					tmpShapeIndex2 = csi2;
				}
			}
		}
	}

	Vector2 contactPoint, contactNormal;

	switch (f.hash())
	{
	case Features::HASH_POINT_SEGMENT:
	{
//		Vector2 sToP = f.f1.point - f.f2.segmentP1;
//		Vector2 dir = crossProductDouble(f.f2.segmentP2 - f.f2.segmentP1, sToP).normalize();
//		contactPoint = f.f1.point - ((dotProduct(dir, sToP) * 0.5f) * dir);

		contactPoint = f.f1.point;
		contactNormal = -normalAt(f.f2.segmentP1, f.f2.segmentP2, pos1.linear).normalize();
		// direction of normal doesnt matter
		//contactNormal = normalVectorCCW(f.f2.segmentP2 - f.f2.segmentP1).normalize();
		break;
	}
	case Features::HASH_SEGMENT_POINT:
	{
//		Vector2 sToP = f.f2.point - f.f1.segmentP1;
//		Vector2 dir = crossProductDouble(f.f1.segmentP2 - f.f1.segmentP1, sToP).normalize();
//		contactPoint = f.f2.point - ((dotProduct(dir, sToP) * 0.5f) * dir);

		contactPoint = f.f2.point;
		contactNormal = normalAt(f.f1.segmentP1, f.f1.segmentP2, pos2.linear).normalize();
		// direction of normal doesnt matter
		//contactNormal = normalVectorCCW(f.f1.segmentP2 - f.f1.segmentP1).normalize();
		break;
	}
	case Features::HASH_SEGMENT_SEGMENT:
	{
		Vector2 v1 = f.f1.segmentP2 - f.f1.segmentP1;
		Vector2 v2 = f.f2.segmentP2 - f.f2.segmentP1;

		if (dotProduct(v1, v2) < 0.0f)
			std::swap(f.f2.segmentP1, f.f2.segmentP2);

		Vector2 v11 = f.f2.segmentP1 - f.f1.segmentP1;
		Vector2 v1n = v1.normalize();

		Vector2 normalDist = crossProductDouble(v1n, v11);

		Vector2 seg1;
		if (dotProduct(v1, v11) >= 0.0f)
		{
			seg1 = f.f2.segmentP1;
			normalDist = -normalDist;
		}
		else
			seg1 = f.f1.segmentP1;

		Vector2 seg2 = v1.lengthSq() <= dotProduct(v1, (f.f2.segmentP2 - f.f1.segmentP1)) ? f.f1.segmentP2 : f.f2.segmentP2;
		Vector2 tangentDist = v1n * dotProduct(v1n, (seg2 - seg1));

		contactPoint = seg1 + 0.5f * normalDist + 0.5f * tangentDist;

		contactNormal = normalAt(f.f1.segmentP1, f.f1.segmentP2, pos2.linear).normalize();
		// direction of normal doesnt matter
		//contactNormal = normalVectorCCW(f.f1.segmentP2 - f.f1.segmentP1).normalize();


		break;
	}
	case Features::HASH_POINT_POINT:
	{
		contactPoint = f.f1.point * 0.5f + f.f2.point * 0.5f;

		// contact normal based on tangential line of collider 1 or 2
		if (collider1.getShape(shapeIndex1).getLocalContactNormal(-pos1.angular * (contactPoint - pos1.linear), contactNormal))
		{
			contactNormal = pos1.angular * contactNormal;
			break;
		}

		if (collider2.getShape(shapeIndex2).getLocalContactNormal(-pos2.angular * (contactPoint - pos2.linear), contactNormal))
		{
			contactNormal = pos2.angular * -contactNormal;
			break;
		}

		//perfectly aligned corners or point masses
		Vector2 r1 = contactPoint - (collider1.getCenterOfMassOffset() + pos1.linear);
		Vector2 r2 = contactPoint - (collider2.getCenterOfMassOffset() + pos2.linear);

		float w1 = collider1.getPartialAngularVelocity(frameTOI);
		float w2 = collider2.getPartialAngularVelocity(frameTOI);

		contactNormal = (collider2.getPartialLinearVelocity(frameTOI) + crossProduct(w2, r2) - collider1.getPartialLinearVelocity(frameTOI) - crossProduct(w1, r1)).normalize();
		break;
	}
	}


	collisionData = { frameTOI, &collider1, &collider2, shapeIndex1, shapeIndex2, contactPoint, contactNormal };
	return true;
};


void PhysicsNarrowPhase::queryCollisions(const float timestep)
{
	const CollisionList& collisionCandidates = broadPhase.queryCollisions();
	if (!collisionCandidates.size()) // no collisions found
		return;

	std::multimap<float, CollisionData> collisions;
	//std::unordered_map<std::pair<Collider*, Collider*>, float> colliderPairs;
	CollisionData cd;

	for (unsigned int i = 0, iMax = (unsigned int)collisionCandidates.size(); i < iMax; ++i)
		if (continuosCollisionCheck(*collisionCandidates[i].first, *collisionCandidates[i].second, timestep, cd))
		{
			collisions.emplace(std::piecewise_construct, std::forward_as_tuple(cd.frameTOI), std::forward_as_tuple(cd));
			//colliderPairs.emplace(std::piecewise_construct, std::forward_as_tuple(cd.collider1, cd.collider2), std::forward_as_tuple(cd.frameTOI));
		}


	// resolve impacts
	while (collisions.size())
	{
		auto it = collisions.begin();
		CollisionData cd = it->second;
		collisions.erase(it);

		Vector2 r1 = cd.contactPoint - (cd.collider1->getCenterOfMassOffset() + cd.collider1->getPartialPosition(cd.frameTOI));
		Vector2 r2 = cd.contactPoint - (cd.collider2->getCenterOfMassOffset() + cd.collider2->getPartialPosition(cd.frameTOI));

		Velocity V1 = cd.collider1->getPartialVelocity(cd.frameTOI);
		Velocity V2 = cd.collider2->getPartialVelocity(cd.frameTOI);

		Vector2 Vr = (V2.linear + crossProduct(V2.angular, r2)) - (V1.linear + crossProduct(V1.angular, r1));

		auto eIt = restitutionCoefficients.find(MaterialPair(cd.collider1->getShapeMaterial(cd.shapeIndex1), cd.collider2->getShapeMaterial(cd.shapeIndex2)));
		float e = DEFAULT_RESTITUTION_COEF;
		if (eIt != restitutionCoefficients.end())
			e = eIt->second;


		//                               -(1+e) * Vr * n
		// jr = -------------------------------------------------------------------------
		//       m1^-1 + m2^-1 + ((I1^-1 * (r1 x n) x r1) + (I2^-1 * (r2 x n) x r2)) * n

		float jrNum = -(1.0f + e) * dotProduct(Vr, cd.collisionNormal);

		Vector2 cpdv = crossProductDouble(r1, cd.collisionNormal, r1) * cd.collider1->getInertiaReciprocal();
		cpdv += crossProductDouble(r2, cd.collisionNormal, r2) * cd.collider2->getInertiaReciprocal();

		float jrDenom = dotProduct(cpdv, cd.collisionNormal);
		jrDenom += cd.collider1->getMassReciprocal();
		jrDenom += cd.collider2->getMassReciprocal();

		float jr = jrNum / jrDenom;
		Vector2 vJr = jr * cd.collisionNormal;

		cd.collider1->applyImpulse(cd.frameTOI, vJr, r1);
		cd.collider2->applyImpulse(cd.frameTOI, -vJr, r2);


		// erase anything that was colliding with the colliders just resolved
		auto cit = collisions.begin(), citEnd = collisions.end();
		while (cit != citEnd)
		{
			auto nextcIt = cit++;
			if ((*nextcIt).second.collider1 == cd.collider1 || (*nextcIt).second.collider1 == cd.collider2 ||
				(*nextcIt).second.collider2 == cd.collider1 || (*nextcIt).second.collider2 == cd.collider2)
				collisions.erase(nextcIt);
		}


		/*
		DirectionVector d;
		Features f;
		closestFeaturesGJK2(cd.collider1->getShape(0), cd.collider1->getPosition(), cd.collider1->getRotation(), cd.collider2->getShape(0), cd.collider2->getPosition(), cd.collider2->getRotation(), d, f);
		if (d.magnitude < 0.0f)
		{
			std::wstring text(1000, '\0');
			swprintf_s(text.data(), 1000, L"collision time: %f, distance after collision: %f, c1 id: %d, c2 id: %d\r\n", cd.frameTOI, d.magnitude, cd.collider1->getPhysicsBroadPhaseId(), cd.collider2->getPhysicsBroadPhaseId());
			OutputDebugString(text.c_str());
		}
		*/

		// update broadphase
		broadPhase.invalidateColliderNode(cd.collider1->getPhysicsBroadPhaseId());
		broadPhase.invalidateColliderNode(cd.collider2->getPhysicsBroadPhaseId());
		broadPhase.update();


		// check for new collisions with both resolved colliders
		CollisionData cdTmp;
		float newFrameTime = -cd.frameTOI;

		// first collider check
		ColliderList newCollisions = broadPhase.queryCollisions(*cd.collider1);
		for (unsigned int i = 0, iMax = (unsigned int)newCollisions.size(); i < iMax; ++i)
			if (continuosCollisionCheck(*cd.collider1, *newCollisions[i], newFrameTime, cdTmp))
				collisions.emplace(std::piecewise_construct, std::forward_as_tuple(cdTmp.frameTOI), std::forward_as_tuple(cdTmp));


		// second collider check
		newCollisions = broadPhase.queryCollisions(*cd.collider2);

		// check for second c1, c2 pair
		if (isOverlapping(cd.collider1->getAABB(), cd.collider2->getAABB()))
		{
			unsigned int i = 0;
			while (newCollisions[i] != cd.collider1) ++i;
			newCollisions[i] = newCollisions.back();
			newCollisions.pop_back();
		}

		for (unsigned int i = 0, iMax = (unsigned int)newCollisions.size(); i < iMax; ++i)
			if (continuosCollisionCheck(*cd.collider2, *newCollisions[i], newFrameTime, cdTmp))
				collisions.emplace(std::piecewise_construct, std::forward_as_tuple(cdTmp.frameTOI), std::forward_as_tuple(cdTmp));

	}

};


#if(DIRECT3D_VERSION == 0x0900)

	void PhysicsNarrowPhase::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght)
	{
		broadPhase.renderTree(_pDevice, _vBuffer, _bufferLenght);

		for (int i = 0, iMax = (int)colliders.size(); i < iMax; ++i)
		{
			//	if (colliding.find(colliders[i].get()) != colliding.end())
			//		colliders[i]->render(_pDevice, _vBuffer, _bufferLenght, 0xFFFFFFFF);
			//	else
			colliders[i]->render(_pDevice, _vBuffer, _bufferLenght);
		}

		if (showDistance)
		{
			struct VertStruct {
				Vector2 xy;
				float z;
				DWORD color;
			} *vertStruct;
			_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);
			unsigned int line = 0;

			Vector2 p1, p2;
			for (unsigned int i = 0, iMax = (unsigned int)colliders.size(); i < iMax; ++i)
			{
				for (unsigned int j = i + 1; j < iMax; ++j)
				{
					if (!closestPoints(*(colliders[i].get()), *colliders[j], p1, p2))
						continue;

					vertStruct[line++] = { p1, 0.0f, 0xFFFFFFFF };
					vertStruct[line++] = { p2, 0.0f, 0xFFFFFFFF };
				}
			}

			_vBuffer->Unlock();
			_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
			_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
			_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, line / 2);

		}
	};

#endif
