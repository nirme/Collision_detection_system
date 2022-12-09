#pragma once
#include "Constants.h"
#include <cmath>
#include <vector>
#include "ColliderShape.h"


class Circle : public ColliderShape
{
private:
	float radius;

protected:
	virtual void updateCenterAndMMOI();

public:
	Circle(Vector2 _center = { 0.0f,0.0f }, float _radius = 0.5f);
	virtual ~Circle();

	virtual AABB getAABB() const;
	virtual AABB getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const;
	virtual Vector2 getLocalPosition() const;
	virtual Vector2 getWorldPosition() const;
	virtual Vector2 getWorldSupport(const Vector2 &d) const;
	virtual Vector2 getLocalSupport(const Vector2 &d) const;

	virtual Vector2 getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;
	virtual Feature getLocalClosestFeature(const Vector2 &direction) const; // local space
	virtual Feature getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const;

	virtual float getMaxDistance(const Vector2 &point) const;

	virtual bool getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const;
	//virtual bool getWorldContactNormal(const Vector2 &contactPoint, Vector2 &normal) const;

	virtual float setDensity(float d);

	void setRadius(float r);
	float getRadius();


#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color);
#endif
};
