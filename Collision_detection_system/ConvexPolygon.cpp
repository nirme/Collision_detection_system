#include "ConvexPolygon.h"



void ConvexPolygon::updateCenterAndMMOI()
{
	float newMMOI;
	Vector2 newCenterOfMass;

	if (getMass() < (float) INFINITY)
		PolygonInertia(getMass(), &points[0], (unsigned int)points.size(), newCenterOfMass, newMMOI);
	else  // centroid calculated independent of mass
		PolygonInertiaFast(getMass(), &points[0], (unsigned int)points.size(), newCenterOfMass, newMMOI);

	setMMOI(newMMOI);
	setCenterOfMass(newCenterOfMass);
};


ConvexPolygon::ConvexPolygon(Vector2 *_points, unsigned int _n) :
	ColliderShape(), 
	points(_points, _points +_n)
{
	for (unsigned int i = 0; i < _n; ++i)
		assert(crossProduct(points[(i + 1) % _n] - points[i], points[(i + 2) % _n] - points[i]) > 0.0f && "Points create concave polygon");
	points.shrink_to_fit();

	//updateCenterAndMMOI();
};


ConvexPolygon::~ConvexPolygon()
{};


AABB ConvexPolygon::getAABB() const
{
	Quaternion rot = getParentRotation();
	Vector2 pos = getParentPosition();

	AABB aabb(rot * points[0]);
	for (int i = 1, iMax = (int)points.size(); i < iMax; ++i)
		aabb += rot * points[i];

	return aabb.move(pos);
};


AABB ConvexPolygon::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	AABB aabb(wRotation * points[0]);
	for (int i = 1, iMax = (int)points.size(); i < iMax; ++i)
		aabb += wRotation * points[i];

	return aabb.move(wPosition);
};


Vector2 ConvexPolygon::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 ConvexPolygon::getWorldPosition() const
{
	// THIS SEEMS WRONG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};


Vector2 ConvexPolygon::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	Quaternion rot = getParentRotation();
	Vector2 dRot = -rot * d;
	float dot = -(float)INFINITY;
	int index = -1;

	for (int i = 0, iMax = (int)points.size(); i < iMax; ++i)
	{
		float testDot = dotProduct(points[i], dRot);
		if (testDot > dot)
		{
			dot = testDot;
			index = i;
		}
	}

	return getParentPosition() + rot * points[index];
	//return getWorldPosition() + rot * points[index];
};


Vector2 ConvexPolygon::getLocalSupport(const Vector2 &d) const
{
	float dot = -(float)INFINITY;
	int index = -1;

	for (int i = 0, iMax = (int)points.size(); i < iMax; ++i)
	{
		float testDot = dotProduct(points[i], d);
		if (testDot > dot)
		{
			dot = testDot;
			index = i;
		}
	}

	return points[index];
};


Vector2 ConvexPolygon::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	Vector2 dRot = -wRotation * direction;
	float dot = -(float)INFINITY;
	int index = -1;

	for (int i = 0, iMax = (int)points.size(); i < iMax; ++i)
	{
		float testDot = dotProduct(points[i], dRot);
		if (testDot > dot)
		{
			dot = testDot;
			index = i;
		}
	}

	return (wPosition + (wRotation * points[index]));
};


Feature ConvexPolygon::getLocalClosestFeature(const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	int i1 = -1, i2 = -1;
	float dist = -(float)INFINITY;

	// loop through all, should be fast enough for a few points
	for (unsigned int i = 0, iMax = (unsigned int)points.size(); i < iMax; ++i)
	{
		float tmpDist = dotProduct(direction, points[i]);
		float diff = tmpDist - dist;

		if (diff > EPSILON)
		{
			dist = tmpDist;
			i1 = i;
			i2 = -1;
		}
		else if (diff >= -EPSILON)
		{
			i2 = i;
		}
	}


	if (i2 == -1)
		return Feature(points[i1]);
	return Feature(points[i1], points[i2]);
};


Feature ConvexPolygon::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	Vector2 dRot = -wRotation * direction;
	int i1 = -1, i2 = -1;
	float dist = -(float)INFINITY;

	// loop through all, should be fast enough for a few points
	for (unsigned int i = 0, iMax = (unsigned int)points.size(); i < iMax; ++i)
	{
		float tmpDist = dotProduct(dRot, points[i]);
		float diff = tmpDist - dist;

		if (diff > EPSILON)
		{
			dist = tmpDist;
			i1 = i;
			i2 = -1;
		}
		else if (diff >= -EPSILON)
		{
			i2 = i;
		}
	}


	if (i2 == -1)
		return Feature(wPosition + (wRotation * points[i1]));
	return Feature(wPosition + (wRotation * points[i1]), wPosition + (wRotation * points[i2]));
};


float ConvexPolygon::getMaxDistance(const Vector2 &point) const
{
	float distanceSq = 0.0f;
	for (unsigned int i = 0, iMax = (unsigned int) points.size(); i < iMax; ++i)
	{
		float tmp = (points[i] - point).lengthSq();
		if (distanceSq < tmp)
			distanceSq = tmp;
	}
	return std::sqrt(distanceSq);
};


bool ConvexPolygon::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	for (unsigned int i2 = 0, iMax = (unsigned int)points.size(), i1 = iMax - 1; i2 < iMax; i1 = i2++)
	{
		Vector2 dir1 = points[i2] - points[i1];
		float d = dotProduct(dir1, contactPoint - points[i1]);
		if (d > EPSILON && (dir1.lengthSq() - d) > EPSILON)
		{
			normal = normalVectorCW(dir1).normalize();
			return true;
		}
	}
	return false;
};


float ConvexPolygon::setDensity(float d)
{
	setMass(PolygonArea(&points[0], (unsigned int) points.size()) * d);
	return getMass();
};


#if(DIRECT3D_VERSION == 0x0900)
	void ConvexPolygon::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
	{
		//DWORD lighting;
		//_pDevice->GetRenderState(D3DRS_LIGHTING, &lighting);
		//_pDevice->SetRenderState(D3DRS_LIGHTING, FALSE);

		struct VertStruct {
			Vector2 xy;
			float z;
			DWORD color;
		} *vertStruct;
		_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);

		Quaternion rot = getParentRotation();
		Vector2 pos = getWorldPosition();

		for (int i = 0, iMax = (int)points.size(); i < iMax; ++i)
		{
			vertStruct[iMax - i - 1] = { rot * points[i] + pos, 0.0f, _color };
		}

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, (UINT)(points.size() - 2));

		//_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);
	};
#endif
