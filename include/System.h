#ifndef System_H
#define System_H

#include "Arduino.h"

class System {
	public:
		System();
		void initialize();
		bool isOn();
		void setConfigState();
		int getConfigState();
	private:
		const int control_pin = 8;             //On Off switch
		const int config_pin = 10;             //Configuation switch
		const int control_ind_pin = 9;         //On off indicator pin
		const int config_ind_1_pin = 11;       //1st macro configuration indicator pin
		const int config_ind_2_pin = 12;       //2nd macro configuration indicator pin
		const int config_ind_3_pin = 13;       //3rd macro configuration indicator pin
		int config_ind_1_state = HIGH;
		int config_ind_2_state = LOW;
		int config_ind_3_state = LOW;
		int config_state;
		int last_config_state = HIGH;
		int config_cycle = 1;
		long config_debounce_delay = 50;
		long last_config_debounce = 0;
};
#endif