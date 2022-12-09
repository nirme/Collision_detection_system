#pragma once
#include "Constants.h"
#include <cmath>
#include "ColliderShape.h"
#include "Math2D.h"


class Ellipse : public ColliderShape
{
private:
	float a, b; // width = 2a, height = 2b
	Quaternion t;
	// x^2/a^2 + y^2/b^2 = 1

protected:
	virtual void updateCenterAndMMOI();

public:
	Ellipse(Vector2 _center = { 0.0f, 0.0f }, float _width = 1.0f, float _height = 0.5f, float _rotationRad = 0.0f);
	virtual ~Ellipse();

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

	virtual float setDensity(float d);

	void setWidth(float width);
	void setHeight(float height);
	void setRotation(float rotation);


#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color);
#endif

};
