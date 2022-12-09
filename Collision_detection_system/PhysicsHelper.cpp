#include "PhysicsHelper.h"


float Determinant(Matrix3 &mx)
{
	return (mx.m11 * (mx.m22*mx.m33 - mx.m23*mx.m32))
		- (mx.m12 * (mx.m21*mx.m33 - mx.m23*mx.m31))
		+ (mx.m13 * (mx.m21*mx.m32 - mx.m22*mx.m31));
};


int Jacobi(float *mx, unsigned int n, float *eigen_values, float *eigen_vectors)
{
	for (unsigned int i = 0; i < n; ++i)
		for (unsigned int j = i + 1; j < n; ++j)
			assert(mx[i*n + j] == mx[j*n + i] && "matrix is not symetric");


	// setup identity
	if (eigen_vectors)
	{
		for (unsigned int i = 0; i < n; ++i)
		{
			eigen_vectors[i*n + i] = 0.0f;
			for (unsigned int j = i + 1; j < n; ++j)
				eigen_vectors[i*n + j] = eigen_vectors[j*n + i] = 1.0f;
		}
	}


	int rotations = 0;
	while (rotations++ < JACOBI_MAX_ROTATION_COUNT)
	{
		// find biggest off diagonal
		unsigned int p, q;
		float max = 0, mt;
		for (unsigned int i = 0; i < n; ++i)
			for (unsigned int j = i + 1; j < n; ++j)
				if ((mt = std::abs(mx[i*n + j])) > max)
				{
					max = mt;
					p = i; q = j;
				}

		// matrix is already diagonal
		if (max <= JACOBI_MACHINE_PRECISION_ZERO)
		{
			if (eigen_values)
				for (unsigned int i = 0; i < n; ++i)
					eigen_values[i] = mx[i*n + i];

			return rotations;
		}


		float theta = (mx[q*n + q] - mx[p*n + p]) / (2 * mx[p*n + q]);
		float t = std::abs(theta) >= MAX_FLOAT_SQRT ?
			0.5f / theta :
			sgn(theta) / (std::abs(theta) + std::sqrt(theta * theta + 1));

		float c = 1 / std::sqrt(t * t + 1);
		float s = t * c;
		float tau = s / (1 + c);


		// a'[pp] = a[pp] - t*a[pq]
		// a'[qq] = a[qq] + t*a[pq]
		float h = t * mx[p*n + q];
		mx[p*n + p] -= h;
		mx[q*n + q] += h;

		// a'[pq] = 0
		// a'[qp] = 0
		mx[p*n + q] = mx[q*n + p] = 0.0f;


		// a'[ip] = a'[pi] = a[ip] ? s(a[iq] + tau*a[ip])
		// a'[iq] = a'[qi] = a[iq] + s(a[ip] ? tau*a[iq])

		for (unsigned int i = 0; i < p; ++i)
		{
			float aip = mx[i*n + p];
			float aiq = mx[i*n + q];
			mx[i*n + p] = mx[p*n + i] = aip - s * (aiq + tau * aip);
			mx[i*n + q] = mx[q*n + i] = aiq + s * (aip - tau * aiq);
		}

		for (unsigned int i = p + 1; i < q; ++i)
		{
			float aip = mx[i*n + p];
			float aiq = mx[i*n + q];
			mx[i*n + p] = mx[p*n + i] = aip - s * (aiq + tau * aip);
			mx[i*n + q] = mx[q*n + i] = aiq + s * (aip - tau * aiq);
		}

		for (unsigned int i = q + 1; i < n; ++i)
		{
			float aip = mx[i*n + p];
			float aiq = mx[i*n + q];
			mx[i*n + p] = mx[p*n + i] = aip - s * (aiq + tau * aip);
			mx[i*n + q] = mx[q*n + i] = aiq + s * (aip - tau * aiq);
		}


		// accumulate eigenvectors if required
		if (eigen_vectors)
		{

			for (unsigned int i = 0; i < n; ++i)
			{
				float evip = eigen_vectors[i*n + p];
				float eviq = eigen_vectors[i*n + q];
				eigen_vectors[i*n + p] = evip * c - eviq * s;
				eigen_vectors[i*n + q] = evip * s + eviq * c;
			}
		}
	}

	return -1;
};


int Jacobi(Matrix3 &mx)
{
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = i + 1; j < 3; ++j)
			assert(mx.m[i * 3 + j] == mx.m[j * 3 + i] && "matrix is not symetric");

	int rotations = 0;
	while (rotations++ < JACOBI_MAX_ROTATION_COUNT)
	{
		unsigned int p = 0, q = 1, r = 2;
		float max = std::abs(mx.m[0 * 3 + 1]), mt;

		if ((mt = std::abs(mx.m[0 * 3 + 2])) > max)
		{
			max = mt;
			p = 0; q = 2; r = 1;
		}

		if ((mt = std::abs(mx.m[1 * 3 + 2])) > max)
		{
			max = mt;
			p = 1; q = 2; r = 0;
		}


		if (max <= JACOBI_MACHINE_PRECISION_ZERO)
			return rotations;


		float theta = (mx.m[q * 3 + q] - mx.m[p * 3 + p]) / (2 * mx.m[p * 3 + q]);
		float t = std::abs(theta) >= MAX_FLOAT_SQRT ?
			0.5f / theta :
			sgn(theta) / (std::abs(theta) + std::sqrt(theta * theta + 1));

		float c = 1 / std::sqrt(t * t + 1);
		float s = t * c;
		float tau = s / (1 + c);


		float h = t * mx.m[p * 3 + q];
		mx.m[p * 3 + p] -= h;
		mx.m[q * 3 + q] += h;

		mx.m[p * 3 + q] = mx.m[q * 3 + p] = 0.0f;

		float aip = mx.m[r * 3 + p];
		float aiq = mx.m[r * 3 + q];
		mx.m[r * 3 + p] = mx.m[p * 3 + r] = aip - s * (aiq + tau * aip);
		mx.m[r * 3 + q] = mx.m[q * 3 + r] = aiq + s * (aip - tau * aiq);
	}

	return -1;
};


Vector2 ComputeCentroid(Vector2 *points, unsigned int n)
{
	float A(0.0f);
	Vector2 r(0.0f);

	for (unsigned int i = 0; i < n; ++i)
	{
		unsigned int j = (i + 1) % n;
		float cp = crossProduct(points[i], points[j]);
		A += cp;
		r.x += points[i].x * points[j].x * cp;
		r.y += points[i].y * points[j].y * cp;

	}
	A *= 0.5f;
	r /= 6 * A;

	return r;
};


void ComputeMassMatrix(Vector2 *points, float *masses, unsigned int n, Vector3 &massMatrixDiagonal, Vector2 &massCenter, float &totalMass)
{
	float m(0.0f);
	Vector2 r(0.0f);
	Matrix3 massMx;

	for (unsigned int i = 0; i < n; ++i)
	{
		m += masses[i];
		r += points[i] * masses[i];

		massMx.m11 += (points[i].y * points[i].y) * masses[i];
		massMx.m22 += (points[i].x * points[i].x) * masses[i];
		massMx.m33 += (points[i].x * points[i].x + points[i].y * points[i].y) * masses[i];
		massMx.m12 += points[i].x * points[i].y * masses[i];
	}

	r /= m;

	massMx.m11 -= m * r.y * r.y;
	massMx.m22 -= m * r.x * r.x;
	massMx.m33 -= m * (r.x * r.x + r.y * r.y);
	massMx.m12 -= m * r.x * r.y;

	Jacobi(massMx); // only 1 rotation needed for 2d

	massMatrixDiagonal.x = massMx.m11;
	massMatrixDiagonal.y = massMx.m22;
	massMatrixDiagonal.z = massMx.m33;
	massCenter = r;
	totalMass = m;
};


float PolygonArea(Vector2 *points, unsigned int n)
{
	float area = 0;
	for (unsigned int i = 0; i < n; ++i)
		area += crossProduct(points[i], points[(i + 1) % n]);
	return std::abs(area) * 0.5f;
};


void PolygonInertia(float mass, Vector2 *points, unsigned int n, Vector2 &centroid, float &mmoi)
{
	float areaDiv = 1.0f / PolygonArea(points, n);

	centroid = { 0.0f };
	mmoi = 0.0f;

	Vector2 &A = points[0];

	for (unsigned int i = 2; i < n; ++i)
	{
		Vector2 &B = points[i - 1];
		Vector2 &C = points[i];
		float partialArea = 0.5f * std::abs(crossProduct(A - B, A - C));
		float partialMass = mass * partialArea * areaDiv;
		Vector2 partialCentroid = (A + B + C) * (1.0f / 3.0f);

		centroid += partialCentroid * partialMass;
	}
	centroid /= mass;

	for (unsigned int i = 2; i < n; ++i)
	{
		Vector2 &B = points[i - 1];
		Vector2 &C = points[i];
		float partialArea = 0.5f * std::abs(crossProduct(A - B, A - C));
		float partialMass = mass * partialArea * areaDiv;
		Vector2 partialCentroid = (A + B + C) * (1.0f / 3.0f);

		Vector2 a = A - partialCentroid;
		Vector2 b = B - partialCentroid;
		Vector2 c = C - partialCentroid;
		float aa = dotProduct(a, a);
		float bb = dotProduct(b, b);
		float cc = dotProduct(c, c);
		float ab = dotProduct(a, b);
		float bc = dotProduct(b, c);
		float ca = dotProduct(c, a);

		float partialMmoi = (aa + bb + cc + ab + bc + ca) * partialMass / 6.0f;

		mmoi += partialMmoi + partialMass * (partialCentroid - centroid).lengthSq();
	}
};


void PolygonInertiaFast(float mass, Vector2 *points, unsigned int n, Vector2 &centroid, float &mmoi)
{
	float Inum(0.0f), Idenom(0.0f);
	float area(0.0f);

	for (unsigned int i = 0; i < n; ++i)
	{
		Vector2 &A = points[i];
		Vector2 &B = points[(i + 1) % n];

		float crossAB = crossProduct(A, B);

		Inum -= crossAB * (dotProduct(A, A) + dotProduct(A, B) + dotProduct(B, B));
		Idenom -= crossAB;
		area += crossAB;
		centroid += (A + B) * crossAB;
	}
	area *= 0.5f;
	centroid /= 6.0f * area;

	// moved to centroid
	mmoi = mass * ((Inum / (6.0f * Idenom)) - centroid.lengthSq());
};
