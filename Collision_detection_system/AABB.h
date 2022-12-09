#pragma once

#include <cassert>
#include "Vector2.h"
#include "Math2D.h"


struct AABB
{
	Vector2 vMin;
	Vector2 vMax;


	AABB();
	AABB(const Vector2& _v);
	AABB(float _minX, float _minY, float _maxX, float _maxY);
	AABB(const Vector2& _vMin, const Vector2& _vMax);
	AABB(const AABB &_box);

	AABB& operator= (const AABB& _box);
	AABB& operator+= (const AABB& _box);
	AABB& operator+= (const Vector2 &_point);

	AABB operator+ (const AABB& _box) const;

	void set(const Vector2& _min, const Vector2& _max);

	AABB merge(const Vector2 &_point) const;
	AABB merge(const AABB &_box) const;

	AABB expandAbs(float _distance) const;
	AABB expandMul(float _factor) const;

	AABB move(const Vector2 &_offset) const;

	bool isContaining(const Vector2& _p) const;
	bool isContaining(const Vector2 &_sectionP1, const Vector2 &_sectionP2);
	bool isContaining(const AABB& _box) const;

	bool isOverlapping(const Vector2 &_sectionP1, const Vector2 &_sectionP2) const;
	bool isOverlapping(const AABB& _box) const;
	bool isOverlapping(const AABB& _box, float _margin) const;

	float area() const;
};

bool isOverlapping(const AABB& _box1, const AABB& _box2);
bool isOverlapping(const AABB& _box1, const AABB& _box2, float _margin);
