#include <TouchNetwork.h>

TouchNetwork t_touch(43);
TouchNetwork i_touch(41);
TouchNetwork m_touch(39);
TouchNetwork r_touch(37);
TouchNetwork p_touch(35);

void setup() {
   Serial.begin(9600);
}

void loop() {
   //Initialize each variable
   t_touch.initialize();
   i_touch.initialize();
   m_touch.initialize();
   r_touch.initialize();
   p_touch.initialize();
   
   //Set the touch state of each finger
   i_touch.setTouchState();
   m_touch.setTouchState();
   r_touch.setTouchState();
   p_touch.setTouchState();
  
   //Print values for debuggin
   Serial.print(i_touch.getTouchState());
   Serial.print(", ");
   Serial.print(m_touch.getTouchState());
   Serial.print(", ");
   Serial.print(r_touch.getTouchState());
   Serial.print(", ");
   Serial.println(p_touch.getTouchState());
 
   //1 Touch Macros
   if(i_touch.getTouchState() == 1 && m_touch.getTouchState() == 0 && r_touch.getTouchState() == 0 && p_touch.getTouchState() == 0)
   {
      Mouse.press(MOUSE_LEFT);
      delay(25);
      Mouse.release(MOUSE_LEFT);
   }
   else if(i_touch.getTouchState() == 0 && m_touch.getTouchState() == 1 && r_touch.getTouchState() == 0 && p_touch.getTouchState() == 0)
   {
      Mouse.press(MOUSE_MIDDLE);
      delay(25);
      Mouse.release(MOUSE_MIDDLE);
   }
   else if(i_touch.getTouchState() == 0 && m_touch.getTouchState() == 0 && r_touch.getTouchState() == 1 && p_touch.getTouchState() == 0)
   {
      Mouse.press(MOUSE_RIGHT);
      delay(25);
      Mouse.release(MOUSE_RIGHT);
   }
    
   //2 Touch Macros
   else if(i_touch.getTouchState() == 1 && m_touch.getTouchState() == 1 && r_touch.getTouchState() == 0 && p_touch.getTouchState() == 0)
   {
      Keyboard.press('1');
      delay(25);
      Keyboard.release('1');
   }
   else if(i_touch.getTouchState() == 0 && m_touch.getTouchState() == 1 && r_touch.getTouchState() == 1 && p_touch.getTouchState() == 0)
   {
      Keyboard.press('2');
      delay(25);
      Keyboard.release('2');
   }
   else if(i_touch.getTouchState() == 0 && m_touch.getTouchState() == 0 && r_touch.getTouchState() == 1 && p_touch.getTouchState() == 1)
   {
      Keyboard.press('3');
      delay(25);
      Keyboard.release('3');
   }
    
   //3 Touch Macros
    
   //4 Touch Macro  
}
