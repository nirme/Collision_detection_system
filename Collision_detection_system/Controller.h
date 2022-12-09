#pragma once

#include <memory>

class ControllerValue
{
public:
	virtual ~ControllerValue() {};
	virtual float get() const = 0;
	virtual void set(float _value) = 0;
};
typedef std::shared_ptr<ControllerValue> SharedControllerValuePtr;


template<class T>
class TTimeframeControllerValue : public ControllerValue
{
protected:
	T *owner;

public:
	TTimeframeControllerValue(T *_owner) : ControllerValue(), owner(_owner)
	{
		assert(owner && "TTimeframeControllerValue must have owner");
	};

	virtual float get() const
	{
		return 0.0f;
	};

	virtual void set(float _value)
	{
		owner->progressTime(_value);
	};
};


class ControllerFunc
{
public:
	virtual ~ControllerFunc() {};
	virtual float calculate(float _rawValue) = 0;
};
typedef std::shared_ptr<ControllerFunc> SharedControllerFuncPtr;


class Controller
{
protected:
	SharedControllerValuePtr sourceValue;
	SharedControllerValuePtr destinationValue;
	SharedControllerFuncPtr transformFunc;

	bool enabled;

public:
	Controller(SharedControllerValuePtr _sourceValue, SharedControllerValuePtr _destinationValue, SharedControllerFuncPtr _transformFunc, bool _enabled = true);

	void setSource(SharedControllerValuePtr _sourceValue);
	void setDestination(SharedControllerValuePtr _destinationValue);
	void setTransformationFunction(SharedControllerFuncPtr _transformFunc);

	void setEnabled(bool _enabled = true);
	bool isEnabled();

	void update();
};
typedef std::shared_ptr<Controller> ControllerPtr;
