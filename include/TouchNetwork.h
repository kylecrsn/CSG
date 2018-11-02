#ifndef TouchNetwork_H
#define TouchNetwork_H

#include "Arduino.h"

class TouchNetwork {
	public:
		TouchNetwork(int pin);
		void initialize();
		void reset();
		bool readTouch();
		void setTouch(bool status);
		bool getTouch();
	private:
		const int t_touch_pin = 43;
		int _pin;
		bool _touch;
};
#endif