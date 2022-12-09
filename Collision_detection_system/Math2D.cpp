#include "Math2D.h"


//  by default V2 is treated like point in 2D
Vector2 operator*(const Matrix3& _m, const Vector2& _v)
{
    return Vector2(
            _m.m11 * _v.x + _m.m12 * _v.y + _m.m13,
            _m.m21 * _v.x + _m.m22 * _v.y + _m.m23
    );
};


//  transform 2D vector
Vector2 transformVector(const Matrix3& _m, const Vector2& _v)
{
    return Vector2(
            _m.m11 * _v.x + _m.m12 * _v.y,
            _m.m21 * _v.x + _m.m22 * _v.y
    );
};


Vector2 operator* (const Quaternion& _q, const Vector2& _v)
{
	// based on nVidia SDK with 3d vetors
	//  x = vx - qz * vy * 2w - 2 * qz * qz * vx
	//  y = vy + qz * vx * 2w - 2 * qz * qz * vy
	float zw2 = _q.z * _q.w * 2.0f;
	float zz2 = _q.z * _q.z * 2.0f;
	return Vector2(
		_v.x - (zw2 * _v.y) - (zz2 * _v.x),
		_v.y + (zw2 * _v.x) - (zz2 * _v.y)
	);
};


Vector2 integrate(const Vector2& p1, const Vector2& v, float dt)
{
	return p1 + v * dt;
};


Quaternion integrate(const Quaternion& q1, float w, float dt)
{
	w *= 0.5f * dt;
	return { q1.w - w * q1.z, q1.z + w * q1.w };
};


Position integrate(const Position& p1, const Velocity& v, float dt)
{
	return Position(integrate(p1.linear, v.linear, dt), integrate(p1.angular, v.angular, dt));
};


Vector2 normalize(const Vector2& _v)
{
	return _v / _v.length();
};


//  default transformation goes scaling -> rotation -> translation
Matrix3 affine2DMatrix(const Vector2 &_scale, const Quaternion &_rotation, const Vector2 &_translation)
{
    //  mx from quaternion
    //  1.0f - (2 * z * z)		2 * z * w				0
    //  -2 * z * w				1.0f - (2 * z * z)		0
    //  0						0						1

    //  full matrix
    //	Sx * rc		Sy * rs		tx
    //	Sx * -rs	Sy * rc		ty
    //	0			0			1


    float rc = 1.0f - (2.0f * _rotation.z * _rotation.z);
    float rs = 2.0f * _rotation.z * _rotation.w;

    return Matrix3(
            _scale.x * rc,		_scale.x * rs,		_translation.x,
            _scale.y * -rs,		_scale.y * rc,		_translation.y,
            0.0f,				0.0f,				1.0f
    );
};


Matrix3 &translateMatrix(Matrix3 &_mx, const Vector2 &_translation)
{
    _mx.m13 += _mx.m11 * _translation.x + _mx.m12 * _translation.y;
    _mx.m23 += _mx.m21 * _translation.x + _mx.m22 * _translation.y;
    _mx.m33 += _mx.m31 * _translation.x + _mx.m32 * _translation.y;

    return _mx;
};


Matrix3& rotationMatrix(Matrix3& _m, Quaternion& _q)
{
	// http://www.songho.ca/opengl/gl_quaternion.html

	float _1m2zz = 1.0f - (2.0f * _q.z * _q.z);
	float _2wz = 2.0f * _q.w * _q.z;
	_m = { _1m2zz, -_2wz, 0.0f, _2wz, _1m2zz, 0.0f, 0.0f, 0.0f, 1.0f };
	return _m;
};


Matrix3 rotationMatrix(Quaternion& _q)
{
	Matrix3 m;
	return rotationMatrix(m, _q);

};


float determinant(const Matrix3 &_mx)
{
    return (
                    _mx.m11*_mx.m22*_mx.m33 +
                    _mx.m12*_mx.m23*_mx.m31 +
                    _mx.m13*_mx.m21*_mx.m32
            ) - (
                    _mx.m11*_mx.m23*_mx.m32 +
                    _mx.m12*_mx.m21*_mx.m33 +
                    _mx.m13*_mx.m22*_mx.m31
            );
};


Matrix3 inverse(const Matrix3 &_mx)
{
    /*
    float det = 1.0f / determinant(_mx);
    return Matrix3(	det * (_mx.m22 * _mx.m33 - _mx.m23 * _mx.m32),
                        det * (_mx.m13 * _mx.m32 - _mx.m12 * _mx.m33),
                        det * (_mx.m12 * _mx.m23 - _mx.m13 * _mx.m22),
                        det * (_mx.m23 * _mx.m31 - _mx.m21 * _mx.m33),
                        det * (_mx.m11 * _mx.m33 - _mx.m13 * _mx.m31),
                        det * (_mx.m13 * _mx.m21 - _mx.m11 * _mx.m23),
                        det * (_mx.m21 * _mx.m32 - _mx.m22 * _mx.m31),
                        det * (_mx.m12 * _mx.m31 - _mx.m11 * _mx.m32),
                        det * (_mx.m11 * _mx.m22 - _mx.m12 * _mx.m21)
    );
    */
    // https://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_3_%C3%97_3_matrices

    float A = _mx.m22 * _mx.m33 - _mx.m23 * _mx.m32;
    float B = _mx.m23 * _mx.m31 - _mx.m21 * _mx.m33;
    float C = _mx.m21 * _mx.m32 - _mx.m22 * _mx.m31;
    float detDiv = 1.0f / ( _mx.m11 * A + _mx.m12 * B + _mx.m13 * C);

    return Matrix3(detDiv * A,
                    detDiv * (_mx.m13 * _mx.m32 - _mx.m12 * _mx.m33),
                    detDiv * (_mx.m12 * _mx.m23 - _mx.m13 * _mx.m22),
                    detDiv * B,
                    detDiv * (_mx.m11 * _mx.m33 - _mx.m13 * _mx.m31),
                    detDiv * (_mx.m13 * _mx.m21 - _mx.m11 * _mx.m23),
                    detDiv * C,
                    detDiv * (_mx.m12 * _mx.m31 - _mx.m11 * _mx.m32),
                    detDiv * (_mx.m11 * _mx.m22 - _mx.m12 * _mx.m21)
    );

};


float dotProduct(const Vector2 &_v0, const Vector2 &_v1)
{
    return _v0.x * _v1.x + _v0.y * _v1.y;
};


float dotProduct(const Quaternion &_q0, const Quaternion &_q1)
{
    return _q0.w * _q1.w + _q0.z * _q1.z;
};


float crossProduct(const Vector2 &_v0, const Vector2 &_v1)
{
	return _v0.x * _v1.y - _v0.y * _v1.x;
};


Vector2 crossProduct(float _z0, const Vector2 &_v1)
{
	return { -_z0 * _v1.y, _z0 * _v1.x };
};


Vector2 crossProductDouble(const Vector2 &_v0, const Vector2 &_v1)
{
	return crossProduct(crossProduct(_v0, _v1), _v0);
};


Vector2 crossProductDouble(const Vector2 &_v0, const Vector2 &_v1, const Vector2 &_v2)
{
	return crossProduct(crossProduct(_v0, _v1), _v2);
};


Vector2 normalVectorCCW(const Vector2 & _v)
{
    return {-_v.y, _v.x};
};


Vector2 normalVectorCW(const Vector2 & _v)
{
    return {_v.y, -_v.x};
};


Vector2 rotateVector(const Vector2 &_v, float _r)
{
    float rc = std::cos(_r);
    float rs = std::sin(_r);
    return Vector2(rc * _v.x + rs * _v.y,
                    -rs * _v.x + rc * _v.y
    );
};


Vector2 normalAtOrigin(const Vector2 &a, const Vector2 &b)
{
	//triple cross with AB x A0 x AB evaluate to  (AB x -A) * CCW(AB) / CCW * sign change
	Vector2 AB = b - a;
	float sign = crossProduct(AB, -a);
	return { sign * -AB.y, sign * AB.x };
};


Vector2 normalAt(const Vector2 &seg1, const Vector2 &seg2, const Vector2 &p)
{
	Vector2 s = seg2 - seg1;
	float sign = crossProduct(s, p - seg1);
	return { sign * -s.y, sign * s.x };
};


Vector2 reflectAcrossNormal(const Vector2 &_v, const Vector2 &_normal)
{
    return (_normal * (2.0f * (dotProduct(_normal, _v)))) - _v;
};


Vector2 reflectAcrossLine(const Vector2 &_v, const Vector2 &_line)
{
    return (_line * (2.0f * (dotProduct(_line, _v) / dotProduct(_line, _line)))) - _v;
};


Vector2 pointOnSegment(const Vector2& s1, const Vector2& s2, float r)
{
	return s1 + (s2 - s1) * r;
};


float degreeToRad(float _r)
{
    return _r * ((float)M_PI / 180.0f);
};


bool onLeftSide(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point)
{
	// TO DO!!!!
	//only cross product required
    return dotProduct(normalVectorCCW(_segmentB - _segmentA), _point - _segmentA) > 0.0f;
};


bool onRightSide(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point)
{
	// TO DO!!!!
	//only cross product required
	return dotProduct(normalVectorCCW(_segmentB - _segmentA), _point - _segmentA) < 0.0f;
};


bool segmentsCrossing(const Vector2 &_segment1A, const Vector2 &_segment1B, const Vector2 &_segment2A, const Vector2 &_segment2B)
{
	Vector2 seg1Normal = normalVectorCCW(_segment1B - _segment1A);
	float seg1to2A = dotProduct(seg1Normal, _segment2A - _segment1A);
	float seg1to2B = dotProduct(seg1Normal, _segment2B - _segment1A);

	Vector2 seg2Normal = normalVectorCCW(_segment1B - _segment1A);
	float seg2to1A = dotProduct(seg2Normal, _segment1A - _segment2A);
	float seg2to1B = dotProduct(seg2Normal, _segment1B - _segment2A);


	if (seg1to2A * seg1to2B > 0.0f || seg2to1A * seg2to1B > 0.0f)
		return false;

	if (seg1to2A * seg1to2B < 0.0f && seg2to1A * seg2to1B < 0.0f ||
		std::abs(seg1to2A + seg1to2B) < EPSILON)
		return true;

	// segments on the same line
	// check if segments overlap
	return (_segment1A.x - _segment2A.x) * (_segment1A.x - _segment2B.x) <= 0.0f ||
			(_segment1B.x - _segment2A.x) * (_segment1B.x - _segment2B.x) <= 0.0f;
};


float distance(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point)
{
    Vector2 segmentVect = _segmentB - _segmentA;
    float projDistSq = dotProduct(segmentVect, _point - _segmentA);

    // behind point A
    if (projDistSq <= 0.0f)
        return (_point - _segmentA).length();

    // behind point B
    if (projDistSq >= segmentVect.lengthSq())
        return (_point - _segmentB).length();

    // between A and B
    return std::abs(dotProduct(normalize(normalVectorCCW(segmentVect)), _point - _segmentA));
};


Vector2 closestPoint(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point)
{
	Vector2 segmentVect = _segmentB - _segmentA;
	float projDistSq = dotProduct(segmentVect, _point - _segmentA);

	if (projDistSq <= 0.0f)
		return _segmentA;

	float lenghtSq = segmentVect.lengthSq();
	if (projDistSq >= lenghtSq)
		return _segmentB;

	return _segmentA + segmentVect * (projDistSq / lenghtSq);
};


bool isConvex(const Vector2 *_points, int _length, bool _CW)
{
	// AB x AC <= 0 for CW
	if (_CW)
	{
		for (int i = 0; i < _length; ++i)
			if (crossProduct(_points[(i + 1) % _length] - _points[i], _points[(i + 2) % _length] - _points[i]) > 0.0f)
				return false;
	}
	// AB x AC >= 0 for CCW
	else //!_CW
	{
		for (int i = 0; i < _length; ++i)
			if (crossProduct(_points[(i + 1) % _length] - _points[i], _points[(i + 2) % _length] - _points[i]) < 0.0f)
				return false;
	}

	return true;
};


std::vector<Vector2> generateHull(std::vector<Vector2> &data)
{
	std::sort(&data[0], &data[0] + data.size(), [](const Vector2 &a, const Vector2 &b) { return a.x < b.x; });

	std::vector<Vector2> hull;
	for (int i = 0, iMax = (int)data.size(); i < iMax; ++i)
	{
		int size = (int)hull.size();
		while (size >= 2 && crossProduct(hull[size - 1] - hull[size - 2], data[i] - hull[size - 2]) >= 0.0f)
		{
			hull.pop_back();
			--size;
		}

		hull.emplace_back(data[i]);
	}

	// last point already on the list
	for (int i = (int)data.size() - 2; i >= 0; --i)
	{
		int size = (int)hull.size();
		while (size >= 2 && crossProduct(hull[size - 1] - hull[size - 2], data[i] - hull[size - 2]) >= 0.0f)
		{
			hull.pop_back();
			--size;
		}

		hull.emplace_back(data[i]);
	}

	// last and first point are the same
	hull.pop_back();

	return hull;
};


Matrix3 generateRotationMatrix(float theta)
{
	float st(std::sin(theta));
	float ct(std::cos(theta));
	return Matrix3(ct, -st, 0.0f, st, ct, 0.0f, 0.0f, 0.0f, 1.0f);
};


Matrix3 generateRotationMatrix(float theta, Vector2 pointOfRotation)
{
	float st(std::sin(theta));
	float ct(std::cos(theta));
	return Matrix3(ct, -st, ct * -pointOfRotation.x + st * pointOfRotation.y + pointOfRotation.x,
		st, ct, st * -pointOfRotation.x + ct * -pointOfRotation.y + pointOfRotation.y,
		0.0f, 0.0f, 1.0f);
};


Matrix3 generateTranslationMatrix(Vector2 position)
{
	return Matrix3(1.0f, 0.0f, position.x, 0.0f, 1.0f, position.y, 0.0f, 0.0f, 1.0f);
};


Matrix3 generateScalingMatrix(Vector2 scale)
{
	return Matrix3(scale.x, 0.0f, 0.0f, 0.0f, scale.y, 0.0f, 0.0f, 0.0f, 1.0f);
};


// https://math.stackexchange.com/questions/1749537/ellipses-farthest-point-to-another-point
Vector2 ellipseFurthestPoint(float a, float b, Vector2 point)
{
	// T4 = (a^2 - b^2)^2
	// T3 = 2 * P.x * a * (b^2 - a^2)
	// T2 = P.x^2 * a^2 + P.y^2 * b^2 - (a^2 - b^2)^2
	// T1 = 2 * P.x * a * (a^2 - b^2)
	// T0 = -(P.x^2 * a^2)
	// T4 * t^4 + T3 * t^3 + T2 * t^2 + T1 * t + T0 = 0

	// for closest point negate T3 and T1

	float a2 = a * a;
	float b2 = b * b;
	float sub_a2_b2 = (a2 - b2);
	float px2 = point.x * point.x;
	float p2xa = 2.0f * point.x * a;

	float T4 = sub_a2_b2 * sub_a2_b2;
	float T3 = p2xa * -sub_a2_b2;
	float T2 = px2 * a2 + point.y * point.y * b2 - T4;
	float T1 = p2xa * sub_a2_b2;
	float T0 = -(px2 * a2);

	float roots[4];
	int rCount = polynomial4Roots(T4, T3, T2, T1, T0, roots);

	Vector2 furthestPoint, tmp;
	float distanceSq = -INFINITY;
	for (int i = 0; i < rCount; ++i)
	{
		float root = std::abs(roots[rCount]);
		if (std::abs(1.0f - root) <= EPSILON) // t = 1
			tmp = { point.x > 0.0f ? -a : a, 0.0f };

		else if (root <= EPSILON) // t = 0
			tmp = { 0.0f, point.y > 0.0f ? -b : b };

		else // 0 < t < 1
			tmp = { (point.x > 0.0f ? -a : a) * root,
				(point.y > 0.0f ? -b : b) * std::sqrt(1.0f - root * root) };

		float tmpLen = (tmp - point).lengthSq();
		if (tmpLen > distanceSq)
		{
			distanceSq = tmpLen;
			furthestPoint = tmp;
		}
	}

	return furthestPoint;
};


Vector2 ellipseFurthestPoint(float a, float b, const Quaternion& rotation, const Vector2& center, const Vector2& point)
{
	// T4 = (a^2 - b^2)^2
	// T3 = 2 * P.x * a * (b^2 - a^2)
	// T2 = P.x^2 * a^2 + P.y^2 * b^2 - (a^2 - b^2)^2
	// T1 = 2 * P.x * a * (a^2 - b^2)
	// T0 = -(P.x^2 * a^2)
	// T4 * t^4 + T3 * t^3 + T2 * t^2 + T1 * t + T0 = 0

	// for closest point negate T3 and T1

	// translate to ellipse center and derotate point
	Vector2 p = -rotation * (point - center);

	float a2 = a * a;
	float b2 = b * b;
	float sub_a2_b2 = (a2 - b2);
	float px2 = p.x * p.x;
	float p2xa = 2.0f * p.x * a;

	float T4 = sub_a2_b2 * sub_a2_b2;
	float T3 = p2xa * -sub_a2_b2;
	float T2 = px2 * a2 + p.y * p.y * b2 - T4;
	float T1 = p2xa * sub_a2_b2;
	float T0 = -(px2 * a2);

	// usualy 2 roots, test which one is furthest
	float roots[4];
	int rCount = polynomial4Roots(T4, T3, T2, T1, T0, roots);

	Vector2 furthestPoint, tmp;
	float distanceSq = -INFINITY;
	for (int i = 0; i < rCount; ++i)
	{
		// abs root, signs are base on point position
		float root = std::abs(roots[i]);
		if (std::abs(1.0f - root) <= EPSILON) // t = 1
			tmp = { p.x > 0.0f ? -a : a, 0.0f };

		else if (root <= EPSILON) // t = 0
			tmp = { 0.0f, p.y > 0.0f ? -b : b };

		else // 0 < t < 1
			tmp = { (p.x > 0.0f ? -a : a) * root,
				(p.y > 0.0f ? -b : b) * std::sqrt(1.0f - root * root) };

		float tmpLen = (tmp - p).lengthSq();
		if (tmpLen > distanceSq)
		{
			distanceSq = tmpLen;
			furthestPoint = tmp;
		}
	}

	// translate and rotate point into the original position of ellipse
	return (rotation * furthestPoint) + center;
};


float ellipseFurthestPointDistance(float a, float b, const Quaternion& rotation, const Vector2& center, const Vector2& point)
{
	// T4 = (a^2 - b^2)^2
	// T3 = 2 * P.x * a * (b^2 - a^2)
	// T2 = P.x^2 * a^2 + P.y^2 * b^2 - (a^2 - b^2)^2
	// T1 = 2 * P.x * a * (a^2 - b^2)
	// T0 = -(P.x^2 * a^2)
	// T4 * t^4 + T3 * t^3 + T2 * t^2 + T1 * t + T0 = 0

	// for closest point negate T3 and T1

	// translate to ellipse center and derotate point
	Vector2 p = -rotation * (point - center);

	float a2 = a * a;
	float b2 = b * b;
	float sub_a2_b2 = (a2 - b2);
	float px2 = p.x * p.x;
	float p2xa = 2.0f * p.x * a;

	float T4 = sub_a2_b2 * sub_a2_b2;
	float T3 = p2xa * -sub_a2_b2;
	float T2 = px2 * a2 + p.y * p.y * b2 - T4;
	float T1 = p2xa * sub_a2_b2;
	float T0 = -(px2 * a2);

	// usualy 2 roots, test which one is furthest
	float roots[4];
	int rCount = polynomial4Roots(T4, T3, T2, T1, T0, roots);

	Vector2 tmp;
	float distanceSq = -INFINITY;
	for (int i = 0; i < rCount; ++i)
	{
		// abs root, signs are base on point position
		float root = std::abs(roots[i]);
		if (std::abs(1.0f - root) <= EPSILON) // t = 1
			tmp = { p.x > 0.0f ? -a : a, 0.0f };

		else if (root <= EPSILON) // t = 0
			tmp = { 0.0f, p.y > 0.0f ? -b : b };

		else // 0 < t < 1
			tmp = { (p.x > 0.0f ? -a : a) * root,
				(p.y > 0.0f ? -b : b) * std::sqrt(1.0f - root * root) };

		float tmpLen = (tmp - p).lengthSq();
		if (tmpLen > distanceSq)
			distanceSq = tmpLen;
	}

	return std::sqrt(distanceSq);
};


float distance(const Line2D& line, const Vector2& point)
{
	return (line.a * point.x + line.b * point.y + line.c) / std::sqrt(line.a * line.a + line.b * line.b);
};


Vector2 projectPoint(const Line2D& line, const Vector2& point)
{
	float plCross = point.x * line.b - point.y * line.a;
	float lenSq = 1.0f / (line.a * line.a + line.b * line.b);
	return Vector2(line.b * plCross - line.a * line.c, -line.a * plCross - line.b * line.c) * lenSq;
};


Line2D lineFromPoints(const Vector2 &point1, const Vector2 &point2)
{
	float a = point1.y - point2.y;
	float b = point2.x - point1.x;
	float c = -(point1.x * a + point1.y * b);
	return Line2D(a, b, c);
};


Line2D lineFromPointNormal(const Vector2 &point, const Vector2 &normal)
{
	return Line2D(point.x, point.y, -dotProduct(point, normal));
};


Line2D rotate(const Line2D& line, const Quaternion& q)
{
	return Line2D(q * Vector2(line.a, line.b), line.c);
};


Line2D move(const Line2D& line, const Vector2& v)
{
	if (line.a < line.b)
		return Line2D(line.a, line.b, -(((-line.c / line.a) + v.x) * line.a + v.y * line.b));
	return Line2D(line.a, line.b, -(v.x * line.a + ((-line.c / line.b) + v.y) * line.b));
};

