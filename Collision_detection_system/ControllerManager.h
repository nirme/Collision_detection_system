#pragma once

#include <memory>
#include <unordered_set>

#include "SingletonTemplate.h"
#include "Controller.h"
#include "ControllerValues.h"


class ControllerManager : public Singleton<ControllerManager>
{
protected:
	typedef std::unordered_set<ControllerPtr> ControllerList;
	//typedef std::unordered_set<ControllerPtr> ControllerList;
	//typedef std::vector<ControllerPtr> ControllerList;

	ControllerList activeControllerList;
	ControllerList controllersToRemove;
	SharedFrameTimeControllerValuePtr frameTimeValue;

public:

	ControllerManager();

	SharedFrameTimeControllerValuePtr getFrameTimeControllerValues();

	ControllerPtr createController(SharedControllerValuePtr _source, SharedControllerValuePtr _destination, SharedControllerFuncPtr _function = nullptr);
	ControllerPtr createFrameTimeController(SharedControllerValuePtr _destination, SharedControllerFuncPtr _function = nullptr);

	void removeController(ControllerPtr _controller, bool async = false);

	void addFrameTime(float _time);
	float getFrameTime();

	float getElapsedTime();

	void setTimeSpeed(float _timeSpeed);
	float getTimeSpeed();

	void updateControllers();
};
