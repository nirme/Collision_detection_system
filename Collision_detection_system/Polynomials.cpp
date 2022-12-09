#include "Polynomials.h"


int polynomial2Roots(float a, float b, float c, float* roots)
{
	int rCount = 0;
	float discriminant = b * b - 4 * a * c;

	if (discriminant > EPSILON) // d > 0
	{
		float dSqrt = std::sqrt(discriminant);
		float a2Rec = 1.0f / (2.0f*a);
		roots[rCount++] = (-b + dSqrt) * a2Rec;
		roots[rCount++] = (-b - dSqrt) * a2Rec;
	}
	else if (std::abs(discriminant) < EPSILON) // d == 0
	{
		roots[rCount++] = -0.5f * (b / a);
	}

	return rCount;
};


int polynomial3Roots(float a3, float a2, float a1, float a0, float* roots)
{
	int rCount = 0;

	// div all by a3
	float a3Div = 1.0f / a3;
	a2 *= a3Div;
	a1 *= a3Div;
	a0 *= a3Div;

	float a2_2 = a2 * a2;

	// Q = (3a1 - a2^2)/9
	float Q = (3 * a1 - a2_2) / 9.0f;
	// R = (9a2a1 - 27a0 - 2a2^3)/54
	float R = (9 * a2 * a1 - 27 * a0 - 2.0f * a2_2 * a2) / 54.0f;

	// D = Q^3 + R^2
	float D = Q * Q * Q + R * R;

	if (D > EPSILON) // D positive - 1 root
	{
		float Dsqrt = std::sqrt(D);
		// S = cbrt(R + sqrt(D))
		// T = cbrt(R - sqrt(D))
		// -1/3 * a2 + (S+T)
		roots[rCount++] = -(a2 / 3.0f) + (std::cbrt(R + Dsqrt) + std::cbrt(R - Dsqrt));
		return rCount;
	}

	if (D >= -EPSILON) // EPSILON >= D >= -EPSILON / D = 0 - 2 roots
	{
		float ST = std::cbrt(R); // S and T are the same
		float neg_a2_by_3 = -(a2 / 3.0f);

		// -1/3 * a2 + (S+T)
		roots[rCount++] = neg_a2_by_3 + (2.0f * ST);

		// -1/3 * a2 - (1/2)(S+T)
		roots[rCount++] = neg_a2_by_3 - ST;

		return rCount;
	}

	// D negative - 3 roots
	// require complex numbers

	float imDsqrt = std::sqrt(std::abs(D));
	std::complex<float> S(R, imDsqrt);
	S = std::pow(S, 1.0f / 3.0f);

	// S = cbrt(R + sqrt(D))
	// T = cbrt(R - sqrt(D))
	// sqrt D is imaginary so T is complex conjugate of S
	std::complex<float> T(S.real(), -S.imag());

	// sum of complex conjugates is 2x real, 0x imag
	// B = S + T
	float B = 2.0f * S.real();
	// subtraction of complex conjugates is 0x real, 2x imag from first number
	// A = S - T
	float A = 2.0f * S.imag();

	// -(a2/3)
	float neg_a2_by_3 = -(a2 / 3.0f);

	// -1/3 * a2 + (S+T)
	roots[rCount++] = neg_a2_by_3 + B;

	// -(a2/3) - 1/2*(S+T)
	float neg_a2_by_3_sub_hafl_B = neg_a2_by_3 - 0.5f * B;

	// 1/2 * i * sqrt(3) * (S-T)
	//float half_sqrt_3_times_A = 0.5f * std::sqrt(3.0f) * A;
	float half_sqrt_3_i_A = 0.86602540378443864676f * A;

	// -(a2/3) - 1/2*(S+T) + 1/2 * i * sqrt(3) * (S-T)
	roots[rCount++] = neg_a2_by_3_sub_hafl_B + half_sqrt_3_i_A;

	// -(a2/3) - 1/2*(S+T) - 1/2 * i * sqrt(3) * (S-T)
	roots[rCount++] = neg_a2_by_3_sub_hafl_B - half_sqrt_3_i_A;

	return rCount;
};



int polynomial4Roots(float a4, float a3, float a2, float a1, float a0, float* roots)
{
	int rCount = 0;

	// divide by a4
	float a4div = 1.0f / a4;

	a3 *= a4div;
	a2 *= a4div;
	a1 *= a4div;
	a0 *= a4div;

	float a3_2 = a3 * a3;
	float _4a0 = 4.0f * a0;

	// x^3 + c2*x^2 + c1*x + c0 = 0
	// c2 = -a2;
	// c1 = a1 * a3 - 4 * a0
	// c0 = 4 * a2 * a0 - a1^2 - a3^2 * a0
	float cubicRoots[3];
	int cubicRCount = polynomial3Roots(1.0f, -a2, (a1 * a3 - _4a0), (_4a0 * a2 - a1 * a1 - a3_2 * a0), cubicRoots);

	float cRoot = cubicRoots[0]; //take first root

	// R^2 = 1/4 * a3^2 - a2 + cRoot
	float Rsqr = 0.25f * a3_2 - a2 + cRoot;
	// if one of the roots gives R=0 and use a different one
	// otherwise end results will be incorrect
	if (std::abs(Rsqr) <= EPSILON && cubicRCount > 1)
		Rsqr = 0.25f * a3_2 - a2 + (cRoot = cubicRoots[1]);

	if (Rsqr > EPSILON) // Rsqr > 0 => 
	{
		float R = std::sqrt(Rsqr);

		// 3/4 * a3^2 - R^2 - 2 * a2
		float P1 = 0.75f * a3_2 - Rsqr - 2.0f * a2;
		// 1/4 * (4 * a3 * a2 - 8 * a1 - a3^3) * R^-1
		float P2 = 0.25f*(4.0f * a3 * a2 - 8.0f*a1 - a3_2  * a3) / R;

		// z1 = -1/4 * a3 + 1/2 * R + 1/2 * D
		// z2 = -1/4 * a3 + 1/2 * R - 1/2 * D
		// D = sqrt(P1 + P2)
		float sum_P1_P2 = P1 + P2;
		if (sum_P1_P2 > EPSILON) // P1 + P2 > 0
		{
			// F1 = -1/4 * a3 + 1/2 * R
			float F1 = -0.25f * a3 + 0.5f * R;
			float Ddiv2 = 0.5f * std::sqrt(sum_P1_P2);
			roots[rCount++] = F1 + Ddiv2;
			roots[rCount++] = F1 - Ddiv2;
		}
		else if (sum_P1_P2 >= -EPSILON) // P1 + P2 = 0
			roots[rCount++] = -0.25f * a3 + 0.5f * R;

		// z3 = -1/4 * a3 - 1/2 * R + 1/2 * E
		// z4 = -1/4 * a3 - 1/2 * R - 1/2 * E
		// E = sqrt(P1 - P2)
		float sub_P1_P2 = P1 - P2;
		if (sub_P1_P2 > EPSILON) // P1 - P2 > 0
		{
			// F2 = -1/4 * a3 - 1/2 * R
			float F2 = -0.25f * a3 - 0.5f * R;
			float Ediv2 = 0.5f * std::sqrt(sub_P1_P2);
			roots[rCount++] = F2 + Ediv2;
			roots[rCount++] = F2 - Ediv2;
		}
		else if (sub_P1_P2 >= -EPSILON) // P1 - P2 = 0
			roots[rCount++] = -0.25f * a3 - 0.5f * R;

		return rCount;
	}

	if (Rsqr >= -EPSILON) // Rsqr = 0
	{
		float R = 0;

		// 3/4 * a3^2 - 2 * a2
		float P1 = 0.75f * a3_2 - 2.0f * a2;
		// 2 * sqrt(cRoot^2 - 4 * a0)
		float P2 = 2.0f * std::sqrt(cRoot * cRoot - _4a0);

		float Ddiv2 = 0.5f * std::sqrt(P1 + P2);
		float Ediv2 = 0.5f * std::sqrt(P1 - P2);

		// roots:
		// z1 = -1/4 * a3 + 1/2 * D
		// z2 = -1/4 * a3 - 1/2 * D
		// z3 = -1/4 * a3 + 1/2 * E
		// z4 = -1/4 * a3 - 1/2 * E

		float F = -0.25f * a3;
		roots[rCount++] = F + Ddiv2;
		roots[rCount++] = F - Ddiv2;
		roots[rCount++] = F + Ediv2;
		roots[rCount++] = F - Ediv2;

		return rCount;
	}

	// Rsqr < 0

	std::complex<float> R(0.0f, std::sqrt(std::abs(Rsqr)));

	// 3/4 * a3^2 - R^2 - 2 * a2
	float P1 = 0.75f * a3_2 - Rsqr - 2.0f * a2;
	// 1/4 * (4 * a3 * a2 - 8 * a1 - a3^3) * R^-1
	std::complex<float> P2 = (0.25f * (4.0f * a3 * a2 - 8.0f * a1 - a3_2 * a3)) * std::pow(R, 2.0f);

	std::complex<float> Ddiv2 = 0.5f * std::sqrt(P1 + P2);
	std::complex<float> Ediv2 = 0.5f * std::sqrt(P1 - P2);

	// roots:
	// z1 = -1/4 * a3 + 1/2 * R + 1/2 * D
	// z2 = -1/4 * a3 + 1/2 * R - 1/2 * D
	// z3 = -1/4 * a3 - 1/2 * R + 1/2 * E
	// z4 = -1/4 * a3 - 1/2 * R - 1/2 * E

	std::complex<float> F1 = -0.25f * a3 + 0.5f * R;
	std::complex<float> F2 = -0.25f * a3 - 0.5f * R;

	std::complex<float> test;

	// take only if real
	test = F1 + Ddiv2;
	if (std::abs(test.imag()) < EPSILON)
		roots[rCount++] = test.real();

	test = F1 - Ddiv2;
	if (std::abs(test.imag()) < EPSILON)
		roots[rCount++] = test.real();

	test = F2 + Ediv2;
	if (std::abs(test.imag()) < EPSILON)
		roots[rCount++] = test.real();

	test = F2 - Ediv2;
	if (std::abs(test.imag()) < EPSILON)
		roots[rCount++] = test.real();

	return rCount;
};
