#pragma once

#include <cmath>
#include "Constants.h"
#include "Vector2.h"


struct Line2D
{
	// ax + by + c = 0
	float a, b, c;

	Line2D(float _a = 1.0f, float _b = 0.0f, float _c = 0.0f) :
		a(_a),
		b(_b),
		c(_c)
	{};

	Line2D(Vector2 _slope, float _c = 0.0f) :
		a(_slope.x),
		b(_slope.y),
		c(_c)
	{};

	Line2D(const Line2D &line) :
		a(line.a),
		b(line.b),
		c(line.c)
	{};

	Line2D& normalize()
	{
		float rec_d = 1.0f / std::sqrt(a * a + b * b);
		a *= rec_d;
		b *= rec_d;
		c *= rec_d;
		return *this;
	}



};