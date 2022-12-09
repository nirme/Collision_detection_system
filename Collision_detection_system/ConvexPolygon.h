#pragma once
#include "Constants.h"
#include <cmath>
#include <iterator>
#include <vector>
#include "ColliderShape.h"
#include "Math2D.h"
#include "PhysicsHelper.h"


class ConvexPolygon : public ColliderShape
{
private:
	std::vector<Vector2> points;

protected:
	virtual void updateCenterAndMMOI();

public:
	ConvexPolygon(Vector2 *_points, unsigned int _n); // CCW
	virtual ~ConvexPolygon();

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

#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color);
#endif
};
