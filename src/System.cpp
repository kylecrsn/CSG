#include "Arduino.h"
#include "System.h"

System::System()
{}

void System::initialize()
{
	pinMode(control_pin, INPUT_PULLUP);
	pinMode(config_pin, INPUT_PULLUP);
	pinMode(control_ind_pin, OUTPUT);
	pinMode(config_ind_1_pin, OUTPUT);
	pinMode(config_ind_2_pin, OUTPUT);
	pinMode(config_ind_3_pin, OUTPUT);
	
	digitalWrite(control_ind_pin, LOW);
	digitalWrite(config_ind_1_pin, LOW);
	digitalWrite(config_ind_2_pin, LOW);
	digitalWrite(config_ind_3_pin, LOW);
}

bool System::isOn()
{
	if(digitalRead(control_pin) == LOW)
	{
		digitalWrite(control_ind_pin, HIGH);
		digitalWrite(config_ind_1_pin, config_ind_1_state);
		digitalWrite(config_ind_2_pin, config_ind_2_state);
		digitalWrite(config_ind_3_pin, config_ind_3_state);
		return true;
	}
	else
	{
		digitalWrite(control_ind_pin, LOW);
		digitalWrite(config_ind_1_pin, LOW);
		digitalWrite(config_ind_2_pin, LOW);
		digitalWrite(config_ind_3_pin, LOW);
		return false;
	}
}

void System::setConfigState()
{
	int config_reading = digitalRead(config_pin);
	
	if (config_reading != last_config_state)
		last_config_debounce = millis();  
		
	if ((millis() - last_config_debounce) > config_debounce_delay) 
	{
		if (config_reading != config_state) 
		{
			config_state = config_reading;
			if (config_state == LOW) 
			{
				if(config_cycle == 1)
				{
					config_ind_1_state = !config_ind_1_state;
					config_ind_2_state = !config_ind_2_state;
				}
				if(config_cycle == 2)
				{
					config_ind_3_state= !config_ind_3_state;
					config_ind_2_state = !config_ind_2_state;
				}
				if(config_cycle == 3)
				{
					config_ind_1_state = !config_ind_1_state;
					config_ind_3_state = !config_ind_3_state;
				}
				config_cycle++;
				if(config_cycle == 4)
					config_cycle = 1;
			}
		}
	}
	digitalWrite(config_ind_1_pin, config_ind_1_state);
	digitalWrite(config_ind_2_pin, config_ind_2_state);
	digitalWrite(config_ind_3_pin, config_ind_3_state);
	last_config_state = config_reading;
}

int System::getConfigState()
{
	return config_cycle;
}