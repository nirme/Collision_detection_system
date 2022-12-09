#include "Segment.h"



void Segment::updateCenterAndMMOI()
{
	//setMMOI((getMass() * width^2) / 12.0f);
	setMMOI((getMass() * halfWidth * halfWidth) / 3.0f);
};


Segment::Segment(Vector2 _position, float _length, float _rotation) :
	ColliderShape(),
	halfWidth(_length * 0.5f),
	rotation(_rotation)
{
	setCenterOfMass(_position);
	updateCenterAndMMOI();
};


Segment::~Segment()
{};


AABB Segment::getAABB() const
{
	Vector2 half = (getParentRotation() * rotation) * Vector2(halfWidth, 0.0f);
	AABB aabb(half);
	aabb = aabb.merge(-half);
	return aabb.move(getWorldPosition());
};


AABB Segment::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	Vector2 half = (wRotation * rotation) * Vector2(halfWidth, 0.0f);
	AABB aabb(half);
	aabb = aabb.merge(-half);
	return aabb.move(wPosition + (wRotation * getLocalPosition()));
};


Vector2 Segment::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 Segment::getWorldPosition() const
{
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};


Vector2 Segment::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");

	Vector2 half = (getParentRotation() * rotation) * Vector2(halfWidth, 0.0f);
	float hd = dotProduct(half, d);

	if (hd > 0.0f)
		return getWorldPosition() + half;
	return  getWorldPosition() - half;
};


Vector2 Segment::getLocalSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");

	Vector2 half = rotation * Vector2(halfWidth, 0.0f);
	float hd = dotProduct(half, d);

	if (hd > 0.0f)
		return getLocalPosition() + half;
	return  getLocalPosition() - half;
};


Vector2 Segment::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	Vector2 half = (wRotation * rotation) * Vector2(halfWidth, 0.0f);
	float hd = dotProduct(half, direction);

	Vector2 pos = wPosition + (wRotation * getLocalPosition());
	if (hd > 0.0f)
		return pos + half;
	return  pos - half;
};


Feature Segment::getLocalClosestFeature(const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	Vector2 l = rotation * Vector2(halfWidth, 0.0f);

	Vector2 dr = getLocalPosition();
	float dld = dotProduct(l, direction);

	if (std::abs(dld) <= EPSILON)
		return Feature(-l + dr, l + dr);
	return Feature((dld > 0.0f ? l : -l) + dr);
};


Feature Segment::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	Quaternion r = wRotation * rotation;
	Vector2 l = r * Vector2(halfWidth, 0.0f);

	Vector2 dr = wPosition + (wRotation * getLocalPosition());
	float dld = dotProduct(l, direction);

	if (std::abs(dld) <= EPSILON)
		return Feature(-l + dr, l + dr);
	return Feature((dld > 0.0f ? l : -l) + dr);
};


float Segment::getMaxDistance(const Vector2 &point) const
{
	Vector2 edge(0.0f, halfWidth);
	edge = rotation * edge;

	float distance1Sq = ((getLocalPosition() + edge) - point).lengthSq();
	float distance2Sq = ((getLocalPosition() - edge) - point).lengthSq();

	return std::sqrt(distance1Sq > distance2Sq ? distance1Sq : distance2Sq);
};


bool Segment::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	Vector2 cp = -rotation * contactPoint;
	if (halfWidth - std::abs(cp.x) > EPSILON)
	{
		normal = rotation * Vector2(0.0f, cp.y >= 0.0f ? 1.0f : -1.0f);
		return true;
	}
	return false;
};


float Segment::setDensity(float d)
{
	setMass(d);
	return d;
};


#if(DIRECT3D_VERSION == 0x0900)
	void Segment::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
	{
		struct VertStruct {
			Vector2 xy;
			float z;
			DWORD color;
		} *vertStruct;
		_vBuffer->Lock(0, 0, (void**)&vertStruct, D3DLOCK_DISCARD);

		constexpr float halfThicness = 0.2f;
		Vector2 rotatedSegment = (getParentRotation() * rotation) * Vector2(halfWidth, halfThicness);
		Vector2 pos = getWorldPosition();

		vertStruct[0] = { Vector2(-rotatedSegment.x, -rotatedSegment.y) + pos, 0.0f, _color };
		vertStruct[1] = { Vector2(-rotatedSegment.x, rotatedSegment.y) + pos, 0.0f, _color };
		vertStruct[2] = { Vector2(rotatedSegment.x, rotatedSegment.y) + pos, 0.0f, _color };
		vertStruct[3] = { Vector2(rotatedSegment.x, -rotatedSegment.y) + pos, 0.0f, _color };

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, 2);
	};
#endif
