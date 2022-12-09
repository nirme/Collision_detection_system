#pragma once

#include "Constants.h"
#include <cassert>
#include <cmath>
#include "Vector2.h"

struct ParabolicBounceData
{
	float x0;
	float height;
	float length;
	float radius;
	float width;
	float velocity;
	float time;
};

struct SinusoidalBounceData
{
	float x0;
	float length;
	float amplitude;
	float height;
	float velocity;
	float time;
};


struct PositionalData
{
	Vector2 position;
	Vector2 movementDirection;
};


// x0 - starting horizontal position 
// h - starting vertical position and maximum height
// l - horizontal distance from max to 0 height (half bounce dist)
// w - arena width
// r - radius of the object
// v - horizontal speed
// t - time after start

PositionalData parabolicBounce(float x0, float h, float l, float r, float w, float v, float t);
PositionalData parabolicBounce(const ParabolicBounceData &data);


PositionalData sinusoidalBounce(float x0, float l, float a, float h, float v, float t);
PositionalData sinusoidalBounce(const SinusoidalBounceData &data);
