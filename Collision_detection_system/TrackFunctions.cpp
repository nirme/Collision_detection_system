#include "TrackFunctions.h"


PositionalData parabolicBounce(float x0, float h, float l, float r, float w, float v, float t)
{
	assert(std::abs(x0) + r <= w / 2 && "starting position out of bounds");

	float a = -((h - r) / (l*l));

	float x = x0 + (t * v);

	float wm2r = w - 2.0f * r;

	float Px = std::fmod(x + 0.5f*wm2r, 2.0f*wm2r);
	float dirx = Px <= wm2r ? 1.0f : -1.0f;
	Px = (Px <= wm2r ? Px : (2.0f*wm2r - Px)) - 0.5f*wm2r;

	float tmp = std::fmod((x + l - x0), (2.0f * l)) - l;
	float Py = a * tmp*tmp + h;

	return {
		Vector2(Px, Py), 
		Vector2(dirx, -2.0f * a * -tmp)
	};
};


PositionalData parabolicBounce(const ParabolicBounceData &data)
{
	return parabolicBounce(data.x0, data.height, data.length, data.radius, data.width, data.velocity, data.time);
};


PositionalData sinusoidalBounce(float x0, float l, float a, float h, float v, float t)
{
	float Px = x0 + v * t;
	float tmp = (2.0f * (float)M_PI * (Px - x0)) / l;

	return {
		Vector2(Px, a * std::sin(tmp) + h),
		Vector2(v >= 0.0f ? 1.0f : -1.0f, (2.0f * (float)M_PI * a * std::cos(tmp)) / l)
	};
};


PositionalData sinusoidalBounce(const SinusoidalBounceData &data)
{
	return sinusoidalBounce(data.x0, data.length, data.amplitude, data.height, data.velocity, data.time);
};
