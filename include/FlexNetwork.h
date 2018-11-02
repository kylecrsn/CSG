#ifndef FlexNetwork_H
#define FlexNetwork_H

#include "Arduino.h"

class FlexNetwork {
	public:
		FlexNetwork(int pin);
		void initialize();
		void reset();
		int readFlex();
		void setFlex(int value);
		int getFlex();
	private:
		const int t_min = 1700;
		const int t_max = 2275;
		const int i_min = 1750;
		const int i_max = 2475;
		const int m_min = 1700;
		const int m_max = 2475;
		const int r_min = 1775;
		const int r_max = 2275;
		const int p_min = 1775;
		const int p_max = 2325;
		const int global_min = 0;
		const int global_max = 1000;
		const int t_flex_pin = A0;
		const int i_flex_pin = A1;
		const int m_flex_pin = A2;
		const int r_flex_pin = A3;
		const int p_flex_pin = A4;
		int _pin;
		int _flex;
};
#endif