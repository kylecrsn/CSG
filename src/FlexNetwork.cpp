#include "Arduino.h"
#include "FlexNetwork.h"

FlexNetwork::FlexNetwork(int pin)
{
	_pin = pin;
	_flex = 0;
}

void FlexNetwork::initialize()
{
	pinMode(_pin, INPUT);
}

void FlexNetwork::reset()
{
	_flex = 0;
}

int FlexNetwork::readFlex()
{
	if(_pin == t_flex_pin)
	{
		_flex = constrain(map(analogRead(_pin), t_min, t_max, global_min, global_max), global_min, global_max);
	}
	else if(_pin == i_flex_pin)
	{
		_flex = constrain(map(analogRead(_pin), i_min, i_max, global_min, global_max), global_min, global_max);
	}
	else if(_pin == m_flex_pin)
	{
		_flex = constrain(map(analogRead(_pin), m_min, m_max, global_min, global_max), global_min, global_max);
	}
	else if(_pin == r_flex_pin)
	{
		_flex = constrain(map(analogRead(_pin), r_min, r_max, global_min, global_max), global_min, global_max);
	}
	else if(_pin == p_flex_pin)
	{
		_flex = constrain(map(analogRead(_pin), p_min, p_max, global_min, global_max), global_min, global_max);
	}
	return _flex;
}

void FlexNetwork::setFlex(int value)
{
	_flex = value;
}

int FlexNetwork::getFlex()
{
	return _flex;
}