#include "ControllerManager.h"


template<>
ControllerManager* Singleton<ControllerManager>::impl = nullptr;


ControllerManager::ControllerManager() :
		frameTimeValue(std::make_shared<FrameTimeControllerValue>(FrameTimeControllerValue(1.0f)))
{};


SharedFrameTimeControllerValuePtr ControllerManager::getFrameTimeControllerValues()
{
	return frameTimeValue;
};


ControllerPtr ControllerManager::createController(SharedControllerValuePtr _source, SharedControllerValuePtr _destination, SharedControllerFuncPtr _function)
{
	ControllerPtr controller = std::make_shared<Controller>(_source, _destination, _function);
	activeControllerList.insert(controller);
	return controller;
};


ControllerPtr ControllerManager::createFrameTimeController(SharedControllerValuePtr _destination, SharedControllerFuncPtr _function)
{
	return createController(getFrameTimeControllerValues(), _destination, _function);
};


void ControllerManager::removeController(ControllerPtr _controller, bool async)
{
	if (async)
		controllersToRemove.insert(_controller);
	else
		activeControllerList.erase(_controller);
};


void ControllerManager::addFrameTime(float _time)
{
	frameTimeValue->set(_time);
};


float ControllerManager::getFrameTime()
{
	return frameTimeValue->get();
};


float ControllerManager::getElapsedTime()
{
	return frameTimeValue->getElapsedTime();
};


void ControllerManager::setTimeSpeed(float _timeSpeed)
{
	frameTimeValue->setTimeSpeed(_timeSpeed);
};


float ControllerManager::getTimeSpeed()
{
	return frameTimeValue->getTimeSpeed();
};


void ControllerManager::updateControllers()
{
	if (controllersToRemove.size())
	{
		for (auto it = controllersToRemove.begin(), itEnd = controllersToRemove.end(); it != itEnd; ++it)
			activeControllerList.erase((*it));

		controllersToRemove.clear();
	}

	for (auto it = activeControllerList.begin(), itEnd = activeControllerList.end(); it != itEnd; ++it)
	{
		(*it)->update();
	}
};
