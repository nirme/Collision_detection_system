#include "PointMass.h"


void PointMass::updateCenterAndMMOI()
{
	setMMOI(0.0f);
};


PointMass::PointMass(Vector2 _position) :
	ColliderShape()
{
	setCenterOfMass(_position);
	updateCenterAndMMOI();
};


PointMass::~PointMass()
{};


AABB PointMass::getAABB() const
{
	return AABB(getWorldPosition());
};


AABB PointMass::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	return AABB(wPosition + (wRotation * getLocalPosition()));
};


Vector2 PointMass::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 PointMass::getWorldPosition() const
{
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};


Vector2 PointMass::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	return getWorldPosition();
};


Vector2 PointMass::getLocalSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	return getLocalPosition();
};


Vector2 PointMass::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	return wPosition + (wRotation * getLocalPosition());
};


Feature PointMass::getLocalClosestFeature(const Vector2 &direction) const
{
	return Feature(getLocalSupport(direction));
};


Feature PointMass::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	return Feature(getSupport(wPosition, wRotation, direction));
};


float PointMass::getMaxDistance(const Vector2 &point) const
{
	return (getLocalPosition() - point).length();
};


bool PointMass::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	return false;
};


float PointMass::setDensity(float d)
{
	setMass(d);
	return d;
};


#if(DIRECT3D_VERSION == 0x0900)
	void PointMass::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
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

		/*
		constexpr float scale = 1.0f;
		int pointsIn1_8 = (int)((circlePoints.size() / 2) / 8);

		for (int i = 0, j = 0, iMax = (int)(circlePoints.size() / 2); i < iMax; i += pointsIn1_8)
			vertStruct[j++] = { Vector2(circlePoints[i * 2], circlePoints[i * 2 + 1]) * scale + pos, 0.0f, _color };

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, 6);
		*/

		constexpr float scale = 10.0f;
		int i = 0;

		vertStruct[i++] = { pos - Vector2(1.0f, 0.0f) * scale, 0.0f, _color };
		vertStruct[i++] = { pos + Vector2(1.0f, 0.0f) * scale, 0.0f, _color };
		vertStruct[i++] = { pos - Vector2(0.0f, 1.0f) * scale, 0.0f, _color };
		vertStruct[i++] = { pos + Vector2(0.0f, 1.0f) * scale, 0.0f, _color };

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_LINELIST, 0, 2);

		//_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);
	};
#endif
