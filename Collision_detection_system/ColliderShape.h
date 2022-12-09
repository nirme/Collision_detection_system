#pragma once
#include <memory>
#include "Vector2.h"
#include "Quaternion.h"
#include "AABB.h"
#include "Feature.h"

#include <d3d9.h>

typedef unsigned short MaterialID;

class Collider;

class ColliderShape
{
private:
	Collider *parent;
	float mmoi;
	float mass;
	Vector2 centerOfMass;
	MaterialID materialId;

protected:
	const Vector2& getParentPosition() const;
	const Quaternion& getParentRotation() const;

	void setMMOI(float _mmoi);
	void setCenterOfMass(Vector2 c);
	virtual void updateCenterAndMMOI() = 0;

public:
	ColliderShape();
	virtual ~ColliderShape();

	void setParent(Collider *_parent);

	void setMass(float _mass);
	virtual float setDensity(float d) = 0; //g/m^2
	float getMass() const;

	Vector2 getCenterOfMass() const; // local space
	float getMMOI() const;

	void setMaterialId(MaterialID id);
	MaterialID getMaterialId() const;

	virtual AABB getAABB() const = 0;
	virtual AABB getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const = 0; // arbitrary space

	// remove virtual, all derived classes return center of mass anyway
	virtual Vector2 getLocalPosition() const = 0; // local space
	virtual Vector2 getWorldPosition() const = 0; // world space
	virtual Vector2 getWorldSupport(const Vector2 &d) const = 0; // world space
	virtual Vector2 getLocalSupport(const Vector2 &d) const = 0; // local space

	Vector2 getPosition(const Vector2 &wPosition, const Quaternion &wRotation) const; // arbitrary space
	virtual Vector2 getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const = 0; // arbitrary space
	virtual Feature getLocalClosestFeature(const Vector2 &direction) const = 0; // local space
	virtual Feature getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const = 0; // arbitrary space

	virtual float getMaxDistance(const Vector2 &point) const = 0; // local space

	virtual bool getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const = 0;
	//virtual bool getWorldContactNormal(const Vector2 &contactPoint, Vector2 &normal) const = 0;

#if(DIRECT3D_VERSION == 0x0900)
	virtual void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color) = 0;
#endif
};

typedef std::unique_ptr<ColliderShape> ColliderShapePtr;
