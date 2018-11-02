// ================================================================
// ===                     GLOBAL VARIABLES                     ===
// ================================================================
const int control_pin = 8;          //On Off switch
const int config_pin = 10;          //Configuation switch
const int control_ind_pin = 9;      //On off indicator pin
const int config_ind_1_pin = 11;    //1st macro configuration indicator pin
const int config_ind_2_pin = 12;    //2nd macro configuration indicator pin
const int config_ind_3_pin = 13;    //3rd macro configuration indicator pin
int config_ind_1_state = HIGH;
int config_ind_2_state = LOW;
int config_ind_3_state = LOW;
int config_state;
int last_config_state = HIGH;
int config_cycle = 1;
long config_debounce_delay = 50;
long last_config_debounce = 0;

void setup() {
  pinMode(config_pin, INPUT_PULLUP);
  pinMode(config_ind_1_pin, OUTPUT);
  pinMode(config_ind_2_pin, OUTPUT);
  pinMode(config_ind_3_pin, OUTPUT);

  // set initial LED state
  digitalWrite(config_ind_1_pin, config_ind_1_state);
  digitalWrite(config_ind_2_pin, config_ind_2_state);
  digitalWrite(config_ind_3_pin, config_ind_3_state);
}

void loop() {
 if(digitalRead(control_pin) == LOW)
  {
    // ================================================================
    // ===                          GLOBAL                          ===
    // ================================================================
    digitalWrite(control_ind_pin, HIGH);

  int config_reading = digitalRead(config_pin);
  if (config_reading != last_config_state)
    last_config_debounce = millis();  
  if ((millis() - last_config_debounce) > config_debounce_delay) 
  {
    if (config_reading != config_state) {
      config_state = config_reading;
    if (config_state == LOW) {
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
}
