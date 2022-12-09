#pragma once
#include <cassert>
#include <cmath>
#include "Vector2.h"
#include "Vector3.h"
#include "Matrix3.h"
#include "Math2D.h"

#define MAX_FLOAT_SQRT					1.0e+10f
#define JACOBI_MACHINE_PRECISION_ZERO	0.0f
#define JACOBI_MAX_ROTATION_COUNT		1000


template <typename T> int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

float Determinant(Matrix3 &mx);

int Jacobi(float *mx, unsigned int n, float *eigen_values = nullptr, float *eigen_vectors = nullptr);
int Jacobi(Matrix3 &mx);

Vector2 ComputeCentroid(Vector2 *points, unsigned int n);

// Impulsebased Dynamic Simulation of Rigid Body System, chapter 6.1
void ComputeMassMatrix(Vector2 *points, float *masses, unsigned int n, Vector3 &massMatrixDiagonal, Vector2 &massCenter, float &totalMass);

float PolygonArea(Vector2 *points, unsigned int n);
void PolygonInertia(float m, Vector2 *points, unsigned int n, Vector2 &centroid, float &mmoi);
void PolygonInertiaFast(float mass, Vector2 *points, unsigned int n, Vector2 &centroid, float &mmoi);
