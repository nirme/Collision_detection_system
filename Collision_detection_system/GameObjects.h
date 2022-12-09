#pragma once

#include "Constants.h"
#include <memory>
#include <math.h>
#include <cmath>
#include "ControllerManager.h"
#include "TrackFunctions.h"
#include "Collider.h"
#include "Circle.h"

class BouncingBall : public ParabolicBounceData
{
private:
	ControllerPtr timeController;
	ColliderSPtr ball;

public:

	BouncingBall();
	~BouncingBall();

	void setupBounceData(float rad, const Vector2 &pos0, float fallDist, float arenaWidth, float horizontalVelocity);
	void progressTime(float frameTime);

	void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght) const;
};

class FloatingBall : public SinusoidalBounceData
{
private:
	ControllerPtr timeController;
	ColliderSPtr ball;

public:

	FloatingBall();
	~FloatingBall();

	void setupFloatData(float rad, const Vector2 &pos0, float waveLength, float waveAmplitude, float horizontalVelocity);
	void progressTime(float frameTime);

	void render(IDirect3DDevice9 *_pDevice, IDirect3DVertexBuffer9 *_vBuffer, unsigned int _bufferLenght) const;
};
