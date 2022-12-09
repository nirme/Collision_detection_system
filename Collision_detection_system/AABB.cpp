#include "AABB.h"


AABB::AABB() :
	vMin(0.0f),
	vMax(0.0f)
{};


AABB::AABB(const Vector2& _v) : 
	vMin(_v),
	vMax(_v)
{};


AABB::AABB(float _minX, float _minY, float _maxX, float _maxY) :
	vMin(_minX, _minY), 
	vMax(_maxX, _maxY)
{
	assert(vMin.x <= vMax.x && vMin.y <= vMax.y && "min/max values mismatch");
};


AABB::AABB(const Vector2& _vMin, const Vector2& _vMax) :
	vMin(_vMin), 
	vMax(_vMax)
{
	assert(vMin.x <= vMax.x && vMin.y <= vMax.y && "min/max values mismatch");
};

AABB::AABB(const AABB &_box) :
	vMin(_box.vMin),
	vMax(_box.vMax)
{};

AABB& AABB::operator= (const AABB& _box)
{
	vMin = _box.vMin;
	vMax = _box.vMax;
	return *this;
};


AABB& AABB::operator+= (const AABB& _box)
{
	if (vMin.x > _box.vMin.x)
		vMin.x = _box.vMin.x;
	if (vMin.y > _box.vMin.y)
		vMin.y = _box.vMin.y;
	if (vMax.x < _box.vMax.x)
		vMax.x = _box.vMax.x;
	if (vMax.y < _box.vMax.y)
		vMax.y = _box.vMax.y;
	return *this;
};


AABB& AABB::operator+= (const Vector2 &_point)
{
	if (_point.x < vMin.x)
		vMin.x = _point.x;
	else if (_point.x > vMax.x)
		vMax.x = _point.x;

	if (_point.y < vMin.y)
		vMin.y = _point.y;
	else if (_point.y > vMax.y)
		vMax.y = _point.y;

	return *this;
};


AABB AABB::operator+ (const AABB& _box) const
{
	return AABB(vMin.x < _box.vMin.x ? vMin.x : _box.vMin.x,
		vMin.y < _box.vMin.y ? vMin.y : _box.vMin.y,
		vMax.x > _box.vMax.x ? vMax.x : _box.vMax.x,
		vMax.y > _box.vMax.y ? vMax.y : _box.vMax.y);
};


void AABB::set(const Vector2& _min, const Vector2& _max)
{
	assert(_min.x <= _max.x && _min.y <= _max.y && "min/max values mismatch");
	vMin = _min;
	vMax = _max;
};


AABB AABB::merge(const Vector2 &_point) const
{
	return AABB(*this) += _point;
};


AABB AABB::merge(const AABB &_box) const
{
	return *this + _box;
};


AABB AABB::expandAbs(float _distance) const
{
	return AABB(vMin - _distance, vMax + _distance);
};


AABB AABB::expandMul(float _factor) const
{
	Vector2 diagonalDiff = (vMax - vMin) * (_factor - 1.0f);
	return AABB(vMin - diagonalDiff, vMax + diagonalDiff);
};


AABB AABB::move(const Vector2 &_offset) const
{
	return AABB(vMin + _offset, vMax + _offset);
};


bool AABB::isContaining(const Vector2& _p) const
{
	return _p.x >= vMin.x &&
		_p.y >= vMin.y &&
		_p.x >= vMax.x &&
		_p.y >= vMax.y;
};


bool AABB::isContaining(const Vector2 &_sectionP1, const Vector2 &_sectionP2)
{
	return isContaining(_sectionP1) && isContaining(_sectionP2);
};


bool AABB::isContaining(const AABB& _box) const
{
	return _box.vMin.x >= vMin.x &&
		_box.vMin.y >= vMin.y &&
		_box.vMax.x <= vMax.x &&
		_box.vMax.y <= vMax.y;
};


bool AABB::isOverlapping(const Vector2 &_sectionP1, const Vector2 &_sectionP2) const
{
	const int INSIDE = 0b0000;
	const int TOP = 0b1000;
	const int BOTTOM = 0b0100;
	const int LEFT = 0b0001;
	const int RIGHT = 0b0010;


	int p1Pos(INSIDE), p2Pos(INSIDE);

	if (_sectionP1.x < vMin.x)
		p1Pos |= LEFT;
	else if (_sectionP1.x > vMax.x)
		p1Pos |= RIGHT;

	if (_sectionP1.y < vMin.y)
		p1Pos |= BOTTOM;
	else if (_sectionP1.x > vMax.x)
		p1Pos |= TOP;

	if (_sectionP2.x < vMin.x)
		p2Pos |= LEFT;
	else if (_sectionP2.x > vMax.x)
		p2Pos |= RIGHT;

	if (_sectionP2.y < vMin.y)
		p2Pos |= BOTTOM;
	else if (_sectionP2.x > vMax.x)
		p2Pos |= TOP;


	if (p1Pos == INSIDE || p2Pos == INSIDE ||  // at least 1 point is contained
		(p1Pos | p2Pos) == (TOP | BOTTOM) || // section goes through center
		(p1Pos | p2Pos) == (LEFT | RIGHT))
		return true;

	if ((p1Pos & p2Pos) != 0b0000) // points share at least one non-visible region
		return false;

	Vector2 p1p2 = _sectionP2 - _sectionP1;
	Vector2 rLimit, lLimit;

	switch (p1Pos)
	{
	case LEFT | TOP:
	{
		rLimit = vMin;
		lLimit = vMax;
		break;
	}
	case LEFT:
	{
		rLimit = vMin;
		lLimit = Vector2(vMin.x, vMax.y);
		break;
	}
	case LEFT | BOTTOM:
	{
		rLimit = Vector2(vMax.x, vMin.y);
		lLimit = Vector2(vMin.x, vMax.y);
		break;
	}
	case BOTTOM:
	{
		rLimit = Vector2(vMax.x, vMin.y);
		lLimit = vMin;
		break;
	}
	case RIGHT | BOTTOM:
	{
		rLimit = vMax;
		lLimit = vMin;
		break;
	}
	case RIGHT:
	{
		rLimit = vMax;
		lLimit = Vector2(vMax.x, vMin.y);
		break;
	}
	case RIGHT | TOP:
	{
		rLimit = Vector2(vMin.x, vMax.y);
		lLimit = Vector2(vMax.x, vMin.y);
		break;
	}
	case TOP:
	{
		rLimit = Vector2(vMin.x, vMax.y);
		lLimit = vMax;
		break;
	}
	};

	if (crossProduct(p1p2, rLimit - _sectionP1) <= 0.0f &&
		crossProduct(lLimit - _sectionP1, p1p2) <= 0.0f)
		return true;
	return false;
};


bool AABB::isOverlapping(const AABB& _box) const
{
	return vMin.x < _box.vMax.x &&
		vMin.y < _box.vMax.y &&
		_box.vMin.x < vMax.x &&
		_box.vMin.y < vMax.y;
};


bool AABB::isOverlapping(const AABB& _box, float _margin) const
{
	return vMin.x < _box.vMax.x + _margin &&
		vMin.y < _box.vMax.y + _margin &&
		_box.vMin.x < vMax.x + _margin &&
		_box.vMin.y < vMax.y + _margin;

};


float AABB::area() const
{
	return (vMax.x - vMin.x) * (vMax.y - vMin.y);
};


bool isOverlapping(const AABB& _box1, const AABB& _box2)
{
	return _box1.isOverlapping(_box2);
};


bool isOverlapping(const AABB& _box1, const AABB& _box2, float _margin)
{
	return _box1.isOverlapping(_box2, _margin);
};
