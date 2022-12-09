#pragma once
#include "Constants.h"
#include <cmath>
#include <vector>
#include "ColliderShape.h"

#include <d3d9.h>
#include <array>


class Pill : public ColliderShape
{
private:
	float halfWidth;
	float radius;
	Quaternion rotation;

protected:
	virtual void updateCenterAndMMOI();

public:
	Pill(Vector2 _center = { 0.0f,0.0f }, float _length = 1.0f, float _radius = 0.5f, float _rotation = 0.0f);
	virtual ~Pill();

	virtual AABB getAABB() const;
	virtual AABB getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const;
	virtual Vector2 getLocalPosition() const;
	virtual Vector2 getWorldPosition() const; // world space
	virtual Vector2 getWorldSupport(const Vector2 &d) const;
	virtual Vector2 getLocalSupport(const Vector2 &d) const;
	virtual Vector2 getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;
	virtual Feature getLocalClosestFeature(const Vector2 &direction) const; // local space
	virtual Feature getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;

	virtual float getMaxDistance(const Vector2 &point) const;

	virtual bool getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const;

	virtual float setDensity(float d); //g/m^2

	void setLength(float _length);
	void setRadius(float _radius);
	void setRotation(float _rotation);

#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color);
#endif
};
