#pragma once

#include "Constants.h"
#include "Vector2.h"
#include "Matrix3.h"
#include "Quaternion.h"
#include "ImpulseStructs.h"
#include "Matrix.h"
#include "Line2D.h"
#include "Polynomials.h"

#include <vector>
#include <algorithm>



// by default V2 is treated like point in 2D
Vector2 operator*(const Matrix3& _m, const Vector2& _v);

// transform 2D vector
Vector2 transformVector(const Matrix3& _m, const Vector2& _v);

// rotate 2D point
Vector2 operator* (const Quaternion& _q, const Vector2& _v);


Vector2 integrate(const Vector2& p1, const Vector2& v, float dt);
Quaternion integrate(const Quaternion& q1, float w, float dt);
Position integrate(const Position& p1, const Velocity& v, float dt);

// normalize vector
Vector2 normalize(const Vector2& _v);

// default transformation goes scaling -> rotation -> translation
Matrix3 affine2DMatrix(const Vector2& _scale, const Quaternion& _rotation, const Vector2& _translation);
Matrix3 &translateMatrix(Matrix3& _mx, const Vector2& _translation);

Matrix3& rotationMatrix(Matrix3& _m, Quaternion& _q);
Matrix3 rotationMatrix(Quaternion& _q);

float determinant(const Matrix3& _mx);

Matrix3 inverse(const Matrix3& _mx);

float dotProduct(const Vector2& _v0, const Vector2& _v1);
float dotProduct(const Quaternion& _q0, const Quaternion& _q1);

float crossProduct(const Vector2 &_v0, const Vector2 &_v1);
Vector2 crossProduct(float _z0, const Vector2 &_v1); // needed for double cross
Vector2 crossProductDouble(const Vector2 &_v0, const Vector2 &_v1);
Vector2 crossProductDouble(const Vector2 &_v0, const Vector2 &_v1, const Vector2 &_v2);

Vector2 normalVectorCCW(const Vector2 & _v);
Vector2 normalVectorCW(const Vector2 & _v);

Vector2 rotateVector(const Vector2 &_v, float _r);

Vector2 normalAtOrigin(const Vector2 &a, const Vector2 &b); // UPPDATE THIS!!!!!!!!!!!!!
Vector2 normalAt(const Vector2 &seg1, const Vector2 &seg2, const Vector2 &p);

Vector2 reflectAcrossNormal(const Vector2 &_v, const Vector2 &_normal);
Vector2 reflectAcrossLine(const Vector2 &_v, const Vector2 &_line);

Vector2 pointOnSegment(const Vector2& s1, const Vector2& s2, float r = 0.5f);

float degreeToRad(float _r);

// NEED CHANGE!!!
bool onLeftSide(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point);
bool onRightSide(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point);
bool segmentsCrossing(const Vector2 &_segment1A, const Vector2 &_segment1B, const Vector2 &_segment2A, const Vector2 &_segment2B);

float distance(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point);
Vector2 closestPoint(const Vector2 &_segmentA, const Vector2 &_segmentB, const Vector2 &_point);

bool isConvex(const Vector2 *_points, int _length, bool _CW = true);

std::vector<Vector2> generateHull(std::vector<Vector2> &data);

template<class _InIt>
std::vector<Vector2> generateHull(_InIt _first, _InIt _last)
{
	std::vector<Vector2> data(_first, _last);
	return generateHull(data);
};


Matrix3 generateRotationMatrix(float theta); //CCW
Matrix3 generateRotationMatrix(float theta, Vector2 pointOfRotation); //CCW
Matrix3 generateTranslationMatrix(Vector2 position);
Matrix3 generateScalingMatrix(Vector2 scale);


Vector2 ellipseFurthestPoint(float a, float b, Vector2 point);
Vector2 ellipseFurthestPoint(float a, float b, const Quaternion& rotation, const Vector2& center, const Vector2& point);
float ellipseFurthestPointDistance(float a, float b, const Quaternion& rotation, const Vector2& center, const Vector2& point);


float distance(const Line2D& line, const Vector2& point);
Vector2 projectPoint(const Line2D& line, const Vector2& point);

Line2D lineFromPoints(const Vector2 &point1, const Vector2 &point2);
Line2D lineFromPointNormal(const Vector2 &point, const Vector2 &normal);

Line2D rotate(const Line2D& line, const Quaternion& q);
Line2D move(const Line2D& line, const Vector2& v);
