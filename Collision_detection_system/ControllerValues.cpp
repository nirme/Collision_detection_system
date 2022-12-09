#include "ControllerValues.h"


FrameTimeControllerValue::FrameTimeControllerValue(float _timeSpeed) :
	ControllerValue(),
	timeSpeed(_timeSpeed),
	elapsedTime(0.0f),
	frameTime(0.0f)
{};


float FrameTimeControllerValue::getElapsedTime()
{
	return elapsedTime;
};


float FrameTimeControllerValue::getTimeSpeed()
{
	return timeSpeed;
};


void FrameTimeControllerValue::setTimeSpeed(float _timeSpeed)
{
	timeSpeed = _timeSpeed;
};


float FrameTimeControllerValue::get() const
{
	return frameTime;
};

void FrameTimeControllerValue::set(float _value)
{
	frameTime = timeSpeed * _value;
	elapsedTime += frameTime;
};
