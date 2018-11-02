#include "Arduino.h"
#include "TouchNetwork.h"

TouchNetwork::TouchNetwork(int pin)
{
	_pin = pin;
	_touch = false;
}

void TouchNetwork::initialize()
{
	if(_pin == t_touch_pin)
	{
		pinMode(_pin, OUTPUT);
		digitalWrite(_pin, LOW);
	}
	else
		pinMode(_pin, INPUT_PULLUP);
}

void TouchNetwork::reset()
{
	_touch = false;
}

bool TouchNetwork::readTouch()
{
	if(digitalRead(_pin) == LOW)
		_touch = true;
	else
		_touch = false;
	return _touch;
}

void TouchNetwork::setTouch(bool status)
{
	_touch = status;
}

bool TouchNetwork::getTouch()
{
	return _touch;
}