#include "Controller.h"


Controller::Controller(SharedControllerValuePtr _sourceValue, SharedControllerValuePtr _destinationValue, SharedControllerFuncPtr _transformFunc, bool _enabled) :
	sourceValue(_sourceValue),
	destinationValue(_destinationValue),
	transformFunc(_transformFunc),
	enabled(_enabled)
{};


void Controller::setSource(SharedControllerValuePtr _sourceValue)
{
	sourceValue = _sourceValue;
};


void Controller::setDestination(SharedControllerValuePtr _destinationValue)
{
	destinationValue = _destinationValue;
};


void Controller::setTransformationFunction(SharedControllerFuncPtr _transformFunc)
{
	transformFunc = _transformFunc;
};


void Controller::setEnabled(bool _enabled)
{
	enabled = _enabled;
};


bool Controller::isEnabled()
{
	return enabled;
};


void Controller::update()
{
	if (enabled)
	{
		destinationValue->set(transformFunc ? transformFunc->calculate(sourceValue->get()) : sourceValue->get());
	}
};
