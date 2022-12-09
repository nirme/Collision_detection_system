#include "Pill.h"


void Pill::updateCenterAndMMOI()
{
	// 2x half circles + rectangle
	// half circle:
	// C = 4/3 * r/PI,0
	// I = m * r^2 * (1/2 - 16/9PI^2)
	// rectangle:
	// C = 0,0
	// I = 1/12 * m * (a^2 + b^2)
	// Ic = sum( mmoi + M * (C - Cc)^2 )
	float mDiv = getMass() / (4 * halfWidth + (float)M_PI * radius);
	float massHalfCircle = (float)M_PI_2 * radius * mDiv;
	float massRectangle = 4 * halfWidth * mDiv;

	float r2 = radius * radius;
	float mmoiHcircle = massHalfCircle * r2 * (0.5f - (16.0f / (9.0f*(float)(M_PI*M_PI))));
	float mmoiSquare = (1.0f / 3.0f) * massRectangle * (r2 + halfWidth * halfWidth);

	float offCentroidSquared = (rotation * Vector2((4 * radius) / (3 * (float)M_PI) + halfWidth, 0.0f)).lengthSq();

	float MMOI = 2.0f * (mmoiHcircle + massHalfCircle * offCentroidSquared) + mmoiSquare;

	setMMOI(MMOI);
};


Pill::Pill(Vector2 _center, float _length, float _radius, float _rotation) :
	ColliderShape(),
	halfWidth(std::abs(_length) * 0.5f),
	radius(std::abs(_radius)),
	rotation(_rotation)
{
	setCenterOfMass(_center);
	updateCenterAndMMOI();
};


Pill::~Pill()
{};


AABB Pill::getAABB() const
{
	Vector2 l = (getParentRotation() * rotation) * Vector2(halfWidth, 0.0f);
	AABB aabb(l);
	aabb += -l;
	aabb = aabb.expandAbs(radius);
	return aabb.move(getWorldPosition());
};


AABB Pill::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	Vector2 l = (wRotation * rotation) * Vector2(halfWidth, 0.0f);
	AABB aabb(l);
	aabb += -l;
	aabb = aabb.expandAbs(radius);
	return aabb.move(wPosition + (wRotation * getLocalPosition()));

};


Vector2 Pill::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 Pill::getWorldPosition() const
{
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};


Vector2 Pill::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	Vector2 l = (getParentRotation() * rotation) * Vector2(halfWidth, 0.0f);
	Vector2 pos = getWorldPosition();
	float dld = dotProduct(l, d);

	if (dld > 0.0f)
		return pos + (d * radius) +  l;
	return pos + (d * radius) - l;
};


Vector2 Pill::getLocalSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	Vector2 l = rotation * Vector2(halfWidth, 0.0f);
	return getLocalPosition() + (dotProduct(l, d) > 0.0f ? l : -l) + (d * radius);
};


Vector2 Pill::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	Vector2 l = (wRotation * rotation) * Vector2(halfWidth, 0.0f);
	Vector2 pos = wPosition + (wRotation * getLocalPosition());
	float dld = dotProduct(l, direction);

	if (dld > 0.0f)
		return pos + (direction * radius) + l;
	return pos + (direction * radius) - l;
};


Feature Pill::getLocalClosestFeature(const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	Vector2 l = rotation * Vector2(halfWidth, 0.0f);
	Vector2 dr = getLocalPosition() + direction * radius;
	float dld = dotProduct(l, direction) / l.length();

	if (std::abs(dld) <= EPSILON)
		return Feature(-l + dr, l + dr);
	return Feature((dld > 0.0f ? l : -l) + dr);
};


Feature Pill::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");
	Quaternion r = wRotation * rotation;
	Vector2 l = r * Vector2(halfWidth, 0.0f);
	Vector2 dr = wPosition + (wRotation * getLocalPosition()) + direction * radius;
	float dld = dotProduct(l, direction) / l.length();

	if (std::abs(dld) <= EPSILON)
		return Feature(-l + dr, l + dr);
	return Feature((dld > 0.0f ? l : -l) + dr);
};


float Pill::getMaxDistance(const Vector2 &point) const
{
	Vector2 edge(0.0f, halfWidth);
	edge = rotation * edge;

	float distance1Sq = ((getLocalPosition() + edge) - point).lengthSq();
	float distance2Sq = ((getLocalPosition() - edge) - point).lengthSq();

	return std::sqrt(distance1Sq > distance2Sq ? distance1Sq : distance2Sq) + radius;
};


bool Pill::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	Vector2 cp = -rotation * contactPoint;

	if (std::abs(cp.x) > halfWidth)
		normal = (rotation * (cp - Vector2(0.0f, cp.x >= 0.0f ? halfWidth : -halfWidth))).normalize();
	else
		normal = rotation * Vector2(0.0f, cp.y >= 0.0f ? 1.0f : -1.0f);

	return true;
};


float Pill::setDensity(float d)
{
	setMass((4.0f * halfWidth + (float)M_PI * radius) * radius * d);
	return getMass();
};


void Pill::setLength(float _length)
{
	halfWidth = _length * 0.5f;
	updateCenterAndMMOI();
};


void Pill::setRadius(float _radius)
{
	radius = _radius;
	updateCenterAndMMOI();
};


void Pill::setRotation(float _rotation)
{
	rotation = Quaternion(_rotation);
	updateCenterAndMMOI();
};


#if(DIRECT3D_VERSION == 0x0900)
	void Pill::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
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

		Quaternion rot = getParentRotation() * rotation;
		Vector2 pos = getWorldPosition();
		Vector2 r = rot * Vector2(halfWidth, 0.0f);
		Vector2 l = -r + pos;
		r += pos;

		int i = 0;
		for (int iMax = (int)(circlePoints.size() / 4); i < iMax; ++i)
			vertStruct[i] = { rot * (Vector2(circlePoints[i * 2], circlePoints[i * 2 + 1]) * radius) + r, 0.0f, _color };

		--i;
		for (int iMax = (int)(circlePoints.size() / 2); i < iMax; ++i)
			vertStruct[i] = { rot * (Vector2(circlePoints[i * 2], circlePoints[i * 2 + 1]) * radius) + l, 0.0f, _color };

		vertStruct[i] = { rot * (Vector2(circlePoints[0], circlePoints[1]) * radius) + l, 0.0f, _color };

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, (UINT)((circlePoints.size() / 2) - 2));

		//_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);
	};
#endif
