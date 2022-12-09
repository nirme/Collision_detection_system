#pragma once

#include "Controller.h"


class FrameTimeControllerValue : public ControllerValue
{
protected:
	float frameTime;
	float elapsedTime;

	float timeSpeed;

public:
	FrameTimeControllerValue(float _timeSpeed = 1.0f);

	float getElapsedTime();
	float getTimeSpeed();
	void setTimeSpeed(float _timeSpeed);

	virtual float get() const;
	virtual void set(float _value);
};
typedef std::shared_ptr<FrameTimeControllerValue> SharedFrameTimeControllerValuePtr;
