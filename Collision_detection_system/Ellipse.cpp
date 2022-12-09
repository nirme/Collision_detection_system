#include "Ellipse.h"



void Ellipse::updateCenterAndMMOI()
{
	// M(a^2+b^2)/4
	setMMOI(getMass() * (a * a + b * b) * 0.25f);
	//center doesn't change
};


Ellipse::Ellipse(Vector2 _center, float _width, float _height, float _rotationRad) :
	ColliderShape(),
	a(_width * 0.5f),
	b(_height * 0.5f),
	t(_rotationRad)
{
	setCenterOfMass(_center);
	updateCenterAndMMOI();
};


Ellipse::~Ellipse()
{};


AABB Ellipse::getAABB() const
{
	Quaternion rot = getParentRotation() * t;

	float sin2n = 2.0f * rot.z * rot.w;
	sin2n *= sin2n;
	float cos2n = 1.0f - sin2n;

	float a2 = a * a;
	float b2 = b * b;

	float x = std::sqrt(a2 * cos2n + b2 * sin2n);
	float y = std::sqrt(b2 * cos2n + a2 * sin2n);

	Vector2 pos = getWorldPosition();
	return AABB(pos.x - x, pos.y - y, pos.x + x, pos.y + y);
};


AABB Ellipse::getAABB(const Vector2 &wPosition, const Quaternion &wRotation) const
{
	Quaternion rot = wRotation * t;

	float sin2n = 2.0f * rot.z * rot.w;
	sin2n *= sin2n;
	float cos2n = 1.0f - sin2n;

	float a2 = a * a;
	float b2 = b * b;

	float x = std::sqrt(a2 * cos2n + b2 * sin2n);
	float y = std::sqrt(b2 * cos2n + a2 * sin2n);

	Vector2 pos = wPosition + (wRotation * getLocalPosition());
	return AABB(pos.x - x, pos.y - y, pos.x + x, pos.y + y);
};


Vector2 Ellipse::getLocalPosition() const
{
	return getCenterOfMass();
};


Vector2 Ellipse::getWorldPosition() const
{
	return getParentPosition() + (getParentRotation() * getLocalPosition());
};


Vector2 Ellipse::getWorldSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	// derotate vector to calculate against
	Quaternion rot = getParentRotation() * t;
	Vector2 drot = -rot * d;

	if (std::abs(drot.y) > EPSILON)
	{
		//drot.normalize(); // single rotation not enough to change length

		// y = mx + c - tangent line to point on edge
		float m = drot.x / -drot.y;

		// from quadratic equation of ellipse and tangent line
		// equation must hold for single point
		float c = std::sqrt(a * a * m * m + b * b);
		if (drot.y < 0.0f)
			c = -c;

		float cdiv = 1.0f / c;
		// tangency point on unrotated ellipse
		Vector2 Ptmp(-(a * a * m * cdiv), b * b * cdiv);

		return getWorldPosition() + (rot * Ptmp);
	}

	// derotated d lies on x axis
	return getWorldPosition() + (rot * Vector2(drot.x > 0.0f ? a : -a, 0.0f));
};


Vector2 Ellipse::getLocalSupport(const Vector2 &d) const
{
	assert(d.isNormalized() || "Vector d not normalized");
	// derotate vector to calculate against
	Vector2 drot = -t * d;

	if (std::abs(drot.y) > EPSILON)
	{
		//drot.normalize(); // single rotation not enough to change length

		// y = mx + c - tangent line to point on edge
		float m = drot.x / -drot.y;

		// from quadratic equation of ellipse and tangent line
		// equation must hold for single point
		float c = std::sqrt(a * a * m * m + b * b);
		if (drot.y < 0.0f)
			c = -c;

		float cdiv = 1.0f / c;
		// tangency point on unrotated ellipse
		Vector2 Ptmp(-(a * a * m * cdiv), b * b * cdiv);

		return getLocalPosition() + (t * Ptmp);
	}

	// derotated d lies on x axis
	return getLocalPosition() + (t * Vector2(drot.x > 0.0f ? a : -a, 0.0f));
};


Vector2 Ellipse::getSupport(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	assert(direction.isNormalized() || "Vector d not normalized");

	Vector2 wPos = wPosition + (wRotation * getLocalPosition());
	// derotate vector to calculate against
	Quaternion rot = wRotation * t;
	Vector2 drot = -rot * direction;

	if (std::abs(drot.y) > EPSILON)
	{
		//drot.normalize(); // single rotation not enough to change length

		// y = mx + c - tangent line to point on edge
		float m = drot.x / -drot.y;

		// from quadratic equation of ellipse and tangent line
		// equation must hold for single point
		float c = std::sqrt(a * a * m * m + b * b);
		if (drot.y < 0.0f)
			c = -c;

		float cdiv = 1.0f / c;
		// tangency point on unrotated ellipse
		Vector2 Ptmp(-(a * a * m * cdiv), b * b * cdiv);

		return wPos + (rot * Ptmp);
	}

	// derotated d lies on x axis
	return wPos + (rot * Vector2(drot.x > 0.0f ? a : -a, 0.0f));
};


Feature Ellipse::getLocalClosestFeature(const Vector2 &direction) const
{
	return Feature(getLocalSupport(direction));
};


Feature Ellipse::getClosestFeature(const Vector2 &wPosition, const Quaternion &wRotation, const Vector2 &direction) const
{
	return Feature(getSupport(wPosition, wRotation, direction));
};


float Ellipse::getMaxDistance(const Vector2 &point) const
{
	return ellipseFurthestPointDistance(a, b, t, getLocalPosition(), point);
};


bool Ellipse::getLocalContactNormal(const Vector2 &contactPoint, Vector2 &normal) const
{
	// move back to origin and derotate
	Vector2 cp = -t * (contactPoint - getLocalPosition());
	Vector2 tangent(-cp.y * a * a, cp.x * b * b);
	normal = (t * crossProductDouble(tangent, cp)).normalize();
	return true;
};


float Ellipse::setDensity(float d)
{
	// area = PI * a * b
	setMass((float)M_PI * a * b * d);
	return getMass();
};


void Ellipse::setWidth(float width)
{
	a = width * 0.5f;
	updateCenterAndMMOI();
};


void Ellipse::setHeight(float height)
{
	b = height * 0.5f;
	updateCenterAndMMOI();
};


void Ellipse::setRotation(float rotation)
{
	t = Quaternion(rotation);
	updateCenterAndMMOI();
};


#if(DIRECT3D_VERSION == 0x0900)
	void Ellipse::render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght, DWORD _color)
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

		Quaternion rot = getParentRotation() * t;
		Vector2 pos = getWorldPosition();

		for (int i = 0, iMax = (int)(circlePoints.size() / 2); i < iMax; ++i)
			vertStruct[i] = { rot * Vector2(circlePoints[i * 2] * a, circlePoints[i * 2 + 1] * b) + pos, 0.0f, _color };

		_vBuffer->Unlock();
		_pDevice->SetStreamSource(0, _vBuffer, 0, sizeof(VertStruct));
		_pDevice->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		_pDevice->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, (UINT) ((circlePoints.size() / 2) - 2));

		//_pDevice->SetRenderState(D3DRS_LIGHTING, lighting);
	};
#endif
