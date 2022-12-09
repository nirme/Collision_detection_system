#pragma once
#include "Constants.h"
#include "ColliderShape.h"
#include "Vector2.h"


class Segment : public ColliderShape
{
protected:
	// length on X axis
	float halfWidth;
	Quaternion rotation;

	void updateCenterAndMMOI();

public:
	Segment(Vector2 _position = { 0.0f, 0.0f }, float _length = 0.0f, float _rotation = 0.0f);
	virtual ~Segment();


	virtual AABB getAABB() const;
	virtual AABB getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const;
	virtual Vector2 getLocalPosition() const;
	virtual Vector2 getWorldPosition() const;
	virtual Vector2 getWorldSupport(const Vector2 &d) const;
	virtual Vector2 getLocalSupport(const Vector2 &d) const;
	virtual Vector2 getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;
	virtual Feature getLocalClosestFeature(const Vector2 &direction) const;
	virtual Feature getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;

	virtual float getMaxDistance(const Vector2 &point) const;

	virtual bool getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const;

	virtual float setDensity(float d); //sets mass

#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color);
#endif
};
