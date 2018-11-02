#include <FlexNetwork.h>

FlexNetwork t_flex(A0);
FlexNetwork i_flex(A1);
FlexNetwork m_flex(A2);
FlexNetwork r_flex(A3);
FlexNetwork p_flex(A4);

void setup() {
   Serial.begin(9600);
   analogReadResolution(12);
}

void loop() {
   //Initialize each variable
   t_flex.initialize();
   i_flex.initialize();
   m_flex.initialize();
   r_flex.initialize();
   p_flex.initialize();
   
   //Set the map & constrain of each finger
   t_flex.setFlexMap();
   i_flex.setFlexMap();
   m_flex.setFlexMap();
   r_flex.setFlexMap();
   p_flex.setFlexMap();
   t_flex.setFlexConstrain();
   i_flex.setFlexConstrain();
   m_flex.setFlexConstrain();
   r_flex.setFlexConstrain();
   p_flex.setFlexConstrain();
     
   //Print values for debugging 
   Serial.print(t_flex.getFlex());
   Serial.print(", ");
   Serial.print(i_flex.getFlex());
   Serial.print(", ");
   Serial.print(m_flex.getFlex());
   Serial.print(", ");
   Serial.print(r_flex.getFlex());
   Serial.print(", ");
   Serial.println(p_flex.getFlex());
     
   //1 Flex Macros
   
   //2 Flex Macros
   
   //3 Flex Macros
   
   //4 Flex Macros
   
   //5 Flex Macro
}
