#include "Circle.h"



void Circle::updateCenterAndMMOI()
{
	setMMOI(0.5f * getMass() * radius * radius);
	//center doesn't change no mater what
};


Circle::Circle(Vector2 _center, float _radius) : 
	ColliderShape(), 
	radius(_radius)
{
	setCenterOfMass(_center);
	updateCenterAndMMOI();
};


Circle::~Circle()
{};


AABB Circle::getAABB() const
{
	Vector2 pos = getWorldPosition();
	return AABB(pos - radius, pos + radius);
};


AABB Circle::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	Vector2 pos = wPosition + (wRotation * getLocalPosition());
	return AABB(pos - radius, pos + radius);
};


Vector2 Circle::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 Circle::getWorldPosition() const
{
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};



Vector2 Circle::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized" );
	return getWorldPosition() + (d * radius);
};


Vector2 Circle::getLocalSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	return getLocalPosition() + (d * radius);
};


Vector2 Circle::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	return (wPosition + (wRotation * getLocalPosition())) + (direction * radius);
};


Feature Circle::getLocalClosestFeature(const Vector2 &direction) const
{
	return Feature(getLocalSupport(direction));
};


Feature Circle::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	return Feature(getSupport(wPosition, wRotation, direction));
};


float Circle::getMaxDistance(const Vector2 &point) const
{
	return (getLocalPosition() - point).length() + radius;
};


bool Circle::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	normal = (contactPoint - getLocalPosition()).normalize();
	return true;
};


/*
bool Circle::getWorldContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	normal = (contactPoint - getWorldPosition()).normalize();
	return true;
};
*/


float Circle::setDensity(float d)
{
	setMass((float)M_PI * radius * radius * d);
	return getMass();
};


void Circle::setRadius(float r)
{
	radius = r;
	updateCenterAndMMOI();
};


float Circle::getRadius()
{
	return radius;
};


#if(DIRECT3D_VERSION == 0x0900)

	void Circle::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
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

		Vector2 pos = getWorldPosition();
		Quaternion q = getParentRotation();
		float zw2 = q.z * q.w * 2.0f;
		float zz2 = q.z * q.z * 2.0f;

		vertStruct[0] = { pos , 0.0f, _color };
		for (int i = 1, iMax = (int)(circlePoints.size() / 2); i < iMax; ++i)
		{
			const Vector2 &p = *reinterpret_cast<const Vector2*>(&circlePoints[i * 2]);
			vertStruct[i] = { Vector2(p.x - (zw2 * p.y) - (zz2 * p.x), p.y + (zw2 * p.x) - (zz2 * p.y)) * radius + pos, 0.0f, _color };
		}

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, (UINT)((circlePoints.size() / 2) - 2));

		//_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);
	};

#endif
