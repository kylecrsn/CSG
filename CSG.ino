#include "System.h"
#include "FlexNetwork.h"
#include "TouchNetwork.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>

// ================================================================
// ===                     SYSTEM VARIABLES                     ===
// ================================================================
System global_sys;
int curr_config_state = 1;
int past_config_state = 1;
int sub_config = 1;

// ================================================================
// ===                   FLEX SENSOR VARIABLES                  ===
// ================================================================
const int t_flex_pin = A0;             //Thumb flex sensor pin
const int i_flex_pin = A1;             //Index flex sensor pin
const int m_flex_pin = A2;             //Middle flex sensor pin
const int r_flex_pin = A3;             //Ring flex sensor pin
const int p_flex_pin = A4;             //Pinky flex sensor pin
const int flex_min = 500;
const int flex_max = 1000;
const int scroll_delay_min = 25;
const int scroll_delay_max = 0;
bool t_flex_past = false;
bool i_flex_past = false;
bool m_flex_past = false;
bool r_flex_past = false;
bool p_flex_past = false;
bool all_false_flex = false;
bool scroll_status = false;
bool other_fingers = false;
int z_motion;

FlexNetwork t_flex(t_flex_pin);
FlexNetwork i_flex(i_flex_pin);
FlexNetwork m_flex(m_flex_pin);
FlexNetwork r_flex(r_flex_pin);
FlexNetwork p_flex(p_flex_pin);

// ================================================================
// ===                  TOUCH SENSOR VARIABLES                  ===
// ================================================================
const int t_touch_pin = 43;            //Thumb touch sensor pin
const int i_touch_pin = 41;            //Index finger touch sensor pin
const int m_touch_pin = 39;            //Middle finger touch sensor pin
const int r_touch_pin = 37;            //Ring finger touch sensor pin
const int p_touch_pin = 35;            //Pinky touch sensor pin
bool i_touch_past = false;
bool m_touch_past = false;
bool r_touch_past = false;
bool p_touch_past = false;

TouchNetwork t_touch(t_touch_pin);
TouchNetwork i_touch(i_touch_pin);
TouchNetwork m_touch(m_touch_pin);
TouchNetwork r_touch(r_touch_pin);
TouchNetwork p_touch(p_touch_pin);

// ================================================================
// ===              IMU VARIABLES/HELPER FUNCTIONS              ===
// ================================================================
MPU6050 mpu;

// MPU control/status
bool dmpReady = false;  				// set true if DMP init was successful
uint8_t mpuIntStatus;   				// holds actual interrupt status byte from MPU
uint8_t devStatus;      				// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; 			   		// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     				// count of all bytes currently in FIFO
uint8_t fifoBuffer[64];	 				// FIFO storage buffer
   
//Orientation/motion
Quaternion q;           				// [w, x, y, z]         quaternion container
VectorInt16 aa;         				// [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     				// [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    				// [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    				// [x, y, z]            gravity vector
float euler[3];         				// [psi, theta, phi]    Euler angle container
float ypr[3];           				// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

const int pitch_max = 65;
const int pitch_min = 10;
const int roll_max = 85;
const int roll_min = 10;
const int mouse_min = 1;
const int mouse_max = 15;
bool pos_x_past = false;
bool neg_x_past = false;
bool pos_y_past = false;
bool neg_y_past = false;
int move_dist;
float yaw_imu = 0;
float pitch_imu = 0;
float roll_imu = 0;
float real_pitch_imu = 0;
float real_roll_imu = 0;
int x_motion = 0;
int y_motion = 0;

//Indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() 
{
    mpuInterrupt = true;
}

//Maps IMU values to mouse values
int axisMotion(float ypr_data, char axis) 
{
   move_dist = (int) ypr_data;

   if(axis == 'x')
   {
      //Map the x-axis value to be within +/- 85 max and +/- 10 min accounting for noise on threshold
      if(move_dist >= (roll_min * -1) && move_dist <= roll_min)
         return 0;
      if (move_dist > roll_max) 
         move_dist = roll_max;
      if (move_dist < (roll_max * -1)) 
         move_dist = roll_max * -1;
      
      //Map move_dist to a linear mapping of mouse movement values
      if(move_dist > roll_min)
         move_dist = map(move_dist, roll_min,  roll_max, mouse_min, mouse_max);
      else
      {
         move_dist = map(move_dist, (roll_min * -1),  (roll_max * -1), (mouse_min * -1), (mouse_max * -1));
         move_dist = move_dist;
      }
      
      return move_dist;
   }
   else if(axis == 'y')
   {
      //Map the y-axis value to be within +/- 65 max and +/- 10 min accounting for noise on threshold
      if(move_dist >= (pitch_min * -1) && move_dist <= pitch_min)
         return 0;
      if (move_dist > pitch_max) 
         move_dist = pitch_max;
      if (move_dist < (pitch_max * -1)) 
         move_dist = pitch_max * -1;
      
      //Map move_dist to a linear mapping of mouse movement values
      if(move_dist > pitch_min)
         move_dist = map(move_dist, pitch_min,  pitch_max, mouse_min, mouse_max);
      else
      {
         move_dist = map(move_dist, (pitch_min * -1),  (pitch_max * -1), (mouse_min * -1), (mouse_max * -1));
         move_dist = move_dist;
      }
      
      return move_dist * -1;
   }
}

void setup() 
{
   Wire.begin();
   Serial.begin(115200);
   Mouse.begin();
   Keyboard.begin();
   analogReadResolution(12);
   
   //Initialize global variables
   global_sys.initialize();
   
   //Initialize flex variables
   t_flex.initialize();
   i_flex.initialize();
   m_flex.initialize();
   r_flex.initialize();
   p_flex.initialize();
   
   //Initialize touch variables
   t_touch.initialize();
   i_touch.initialize();
   m_touch.initialize();
   r_touch.initialize();
   p_touch.initialize();
   
   //Initialize IMU variables
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
   while (Serial.available() && Serial.read());
   devStatus = mpu.dmpInitialize();
   mpu.setXAccelOffset(-4570);
   mpu.setYAccelOffset(126);
   mpu.setZAccelOffset(630);
   mpu.setXGyroOffset(-1438);
   mpu.setYGyroOffset(-46);
   mpu.setZGyroOffset(-23);
   if (devStatus == 0) 
   {
      mpu.setDMPEnabled(true);
      attachInterrupt(2, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
   }
}

void loop() 
{
   //Check if on/off switch is "on"
   if(global_sys.isOn())
   {
      //If a config change occurs, reset past values
      global_sys.setConfigState();
      curr_config_state = global_sys.getConfigState();
      if(curr_config_state != past_config_state)
      {
         t_flex.reset();
         i_flex.reset();
         m_flex.reset();
         r_flex.reset();
         p_flex.reset();
         i_touch.reset();
         m_touch.reset();
         r_touch.reset();
         p_touch.reset();
         yaw_imu = 0;
         pitch_imu = 0;
         roll_imu = 0;
         x_motion = 0;
         y_motion = 0;
         z_motion = 0;
         t_flex_past = false;
         i_flex_past = false;
         m_flex_past = false;
         r_flex_past = false;
         p_flex_past = false;
         all_false_flex = false;
         scroll_status = false;
         i_touch_past = false;
         m_touch_past = false;
         r_touch_past = false;
         p_touch_past = false;
         pos_x_past = false;
         neg_x_past = false;
         pos_y_past = false;
         neg_y_past = false;
         sub_config = 1;
         if(other_fingers == true)
         {
            Keyboard.release('8');
            other_fingers = false;
         }
         past_config_state = curr_config_state;
      }
      
      // ================================================================
      // ================================================================
      // ===                         MAPPINGS                         ===
      // ================================================================
      // ================================================================
      
      // ================================================================
      // ===                        FLEX SENSOR                       ===
      // ================================================================
      //Set the flex state each finger
      if(t_flex_past == false)
         t_flex.readFlex();
      else
         t_flex.setFlex(1000);
      if(i_flex_past == false)
         i_flex.readFlex();
      else
         i_flex.setFlex(1000);
      if(m_flex_past == false)
         m_flex.readFlex();
      else
         m_flex.setFlex(1000);
      if(r_flex_past == false)
         r_flex.readFlex();
      else
         r_flex.setFlex(1000);
      if(p_flex_past == false)
         p_flex.readFlex();
      else
         p_flex.setFlex(1000);
    
      // ================================================================
      // ===                       TOUCH SENSOR                       ===
      // ================================================================
      //Set the touch state of each finger
      if(i_touch_past == false)
         i_touch.readTouch();
      else
         i_touch.setTouch(true);
      if(m_touch_past == false)
         m_touch.readTouch();
      else
         m_touch.setTouch(true);
      if(r_touch_past == false)
         r_touch.readTouch();
      else
         r_touch.setTouch(true);
      if(p_touch_past == false)
         p_touch.readTouch();
      else
         p_touch.setTouch(true);
      
      // ================================================================
      // ===                        IMU SENSOR                        ===
      // ================================================================
      if(!dmpReady)
         return;
         
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();
      
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
      {
         //Reset so we can continue cleanly
         mpu.resetFIFO();
         Serial.println(F("FIFO OVERFLOW"));
      } 
      else if (mpuIntStatus & 0x02) 
      {
         while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
         
         mpu.getFIFOBytes(fifoBuffer, packetSize);
         fifoCount -= packetSize;
         
         //Convert values to ypr format
         mpu.dmpGetQuaternion(&q, fifoBuffer);
         mpu.dmpGetGravity(&gravity, &q);
         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         
         if(gravity.z>0)
            ypr[1] = atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
         else if(gravity.z<0)
            ypr[1] =  - (PI)-atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));     
         
         if(gravity.z>0) 
           ypr[2] = atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));
         else if(gravity.z<0)
           ypr[2] = -PI - atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));
           
         ypr[0] = ypr[0] * 180/M_PI;
         ypr[1] = ypr[1] * 180/M_PI;
         ypr[2] = ypr[2] * 180/M_PI;
         
         yaw_imu = ypr[0];
         if(ypr[1] > (pitch_max * -1) &&  ypr[1] < pitch_max)
            pitch_imu = ypr[1];
         if(ypr[2] > (roll_max * -1) &&  ypr[2] < roll_max)
            roll_imu = ypr[2];
         
         real_pitch_imu = pitch_imu;
         real_roll_imu = roll_imu;
      }
      
      // ================================================================
      // ================================================================
      // ===                          MACROS                          ===
      // ================================================================
      // ================================================================
      
      // ================================================================
      // ===                     CONFIGURATION 1                      ===
      // ================================================================
      if(curr_config_state == 1)
      {
         // ================================================================
         // ===                        FLEX SENSOR                       ===
         // ================================================================
         //1 Flex Macros
   
         //2 Flex Macros
         
         //3 Flex Macros
         if(m_flex.getFlex() >= 500 && r_flex.getFlex() >= 500 && p_flex.getFlex() == 0)
         {
            scroll_status = true;
            if(t_flex.getFlex() >= 500 && i_flex.getFlex() <= 500)
            {
               z_motion = 1;
               Mouse.move(0, 0, z_motion);
               delay(25);
            }
            else if(t_flex.getFlex() <= 500 && i_flex.getFlex() >= 500)
            {
               z_motion = -1;
               Mouse.move(0, 0, z_motion);
               delay(25);
            }
            else
            {
               z_motion = 0;
               Mouse.move(0, 0, z_motion);
            }   
         }
         else
            scroll_status = false;
            
         //4 Flex Macros
         
         //5 Flex Macros
         
         // ================================================================
         // ===                       TOUCH SENSOR                       ===
         // ================================================================
         //1 Touch Macros
         if(i_touch.getTouch() == true && m_touch.getTouch() == false && r_touch.getTouch() == false && p_touch.getTouch() == false)
         {  
            if(i_touch_past == false)
            {
               i_touch_past = true;
               Mouse.press(MOUSE_LEFT);
            }
            else if(i_touch_past == true && i_touch.readTouch() == false)
            {
               i_touch_past = false;
               Mouse.release(MOUSE_LEFT);
            }
         }
         else if(i_touch.getTouch() == false && m_touch.getTouch() == true && r_touch.getTouch() == false && p_touch.getTouch() == false)
         {
            if(m_touch_past == false)
            {
               m_touch_past = true;
               Mouse.press(MOUSE_MIDDLE);
            }
            else if(m_touch_past == true && m_touch.readTouch() == false)
            {
               m_touch_past = false;
               Mouse.release(MOUSE_MIDDLE);
            }
         }
         else if(i_touch.getTouch() == false && m_touch.getTouch() == false && r_touch.getTouch() == true && p_touch.getTouch() == false)
         {
            if(r_touch_past == false)
            {
               r_touch_past = true;
               Mouse.press(MOUSE_RIGHT);
            }
            else if(r_touch_past == true && r_touch.readTouch() == false)
            {
               r_touch_past = false;
               Mouse.release(MOUSE_RIGHT);
            }
         }
         
         //2 Touch Macros
         
         //3 Touch Macros
         
         //4 Touch Macro
         
         // ================================================================
         // ===                        IMU SENSOR                        ===
         // ================================================================
         if(scroll_status == false)
         {
            x_motion = axisMotion(roll_imu, 'x');
            y_motion = axisMotion(pitch_imu, 'y');
            Mouse.move(x_motion, y_motion, 0);
         }
         else
         {
            x_motion = 0;
            y_motion = 0;
         }
      }
      
      // ================================================================
      // ===                     CONFIGURATION 2                      ===
      // ================================================================
      else if(curr_config_state == 2)
      {
         if(t_flex.getFlex() >= 500 && i_flex.getFlex() <= 250 && m_flex.getFlex() <= 250 && r_flex.getFlex() <= 250 && p_flex.getFlex() <= 250)
         {
            if(t_flex_past == false)
            {
               t_flex_past = true;
               sub_config++;
            }
            else if(t_flex_past == true && t_flex.readFlex() <= 250)
            {
               t_flex_past = false;
            }
         }
         if(sub_config > 3)
            sub_config = 1;
            
         if(sub_config == 1)
         {     
            // ================================================================
            // ===                        FLEX SENSOR                       ===
            // ================================================================
            //1 Flex Macros
            
            //2 Flex Macros
            
            //3 Flex Macros
        
            //4 Flex Macros
            
            //5 Flex Macros
            if(t_flex.getFlex() <= 250 && i_flex.getFlex() <= 250 && m_flex.getFlex() <= 250 && r_flex.getFlex() <= 250 && p_flex.getFlex() <= 250 && other_fingers == false)
            {      
               Keyboard.press('8');
               other_fingers = true;
            }
            
            if(i_flex.getFlex() <= 250 && m_flex.getFlex() <= 250 && r_flex.getFlex() >= 500 && p_flex.getFlex() >= 500)
            {  
               Keyboard.release('8');
               if(r_flex_past == false && p_flex_past == false)
               {
                  r_flex_past = true;
                  p_flex_past = true;
                  Keyboard.press('7');
               }
               else if((r_flex_past == true && r_flex.readFlex() <= 250) || (p_flex_past == true && p_flex.readFlex() <= 250))
               {
                  r_flex_past = false;
                  p_flex_past = false;
                  Keyboard.release('7');
                  other_fingers = false;
               }
            }

            if(i_flex.getFlex() >= 500 && m_flex.getFlex() >= 500 && r_flex.getFlex() >= 500 && p_flex.getFlex() >= 500)
            {
               Keyboard.release('8');
               if(i_flex_past == false && m_flex_past == false && r_flex_past == false && p_flex_past == false)
               {
                  Serial.print("b");
                  i_flex_past = true;
                  m_flex_past = true;
                  r_flex_past = true;
                  p_flex_past = true;
                  Keyboard.press('9');
               }
               else if((i_flex_past == true && i_flex.readFlex() <= 250) || (m_flex_past == true && m_flex.readFlex() <= 250) || (r_flex_past == true && r_flex.readFlex() <= 250) || (p_flex_past == true && p_flex.readFlex() <= 250))
               {
                  i_flex_past = false;
                  m_flex_past = false;
                  r_flex_past = false;
                  p_flex_past = false;
                  Keyboard.release('9');
                  other_fingers = false;
               }
            }
         }
         else if(sub_config == 2)
         {
            // ================================================================
            // ===                       TOUCH SENSOR                       ===
            // ================================================================
            if(other_fingers == true)
            {
               Keyboard.release('8');
               other_fingers = false;
            }
            if(i_touch.getTouch() == true && m_touch.getTouch() == false && r_touch.getTouch() == false && p_touch.getTouch() == false)
            {  
               if(i_touch_past == false)
               {
                  i_touch_past = true;
                  delay(15);
                  Keyboard.press('1');
               }
               else if(i_touch_past == true && i_touch.readTouch() == false)
               {
                  i_touch_past = false;
                  Keyboard.release('1');
                  delay(10);
               }
            }
            else if(i_touch.getTouch() == false && m_touch.getTouch() == true && r_touch.getTouch() == false && p_touch.getTouch() == false)
            {
               if(m_touch_past == false)
               {
                  m_touch_past = true;
                  delay(10);
                  Keyboard.press('2');
               }
               else if(m_touch_past == true && m_touch.readTouch() == false)
               {
                  m_touch_past = false;
                  Keyboard.release('2');
                  delay(10);
               }
            }
            else if(i_touch.getTouch() == false && m_touch.getTouch() == false && r_touch.getTouch() == true && p_touch.getTouch() == false)
            {
               if(r_touch_past == false)
               {
                  r_touch_past = true;
                  delay(10);
                  Keyboard.press('3');
               }
               else if(r_touch_past == true && r_touch.readTouch() == false)
               {
                  r_touch_past = false;
                  Keyboard.release('3');
                  delay(10);
               }
            }
            else if(i_touch.getTouch() == false && m_touch.getTouch() == false && r_touch.getTouch() == false && p_touch.getTouch() == true)
            {
               if(p_touch_past == false)
               {
                  p_touch_past = true;
                  delay(10);
                  Keyboard.press('4');
               }
               else if(p_touch_past == true && p_touch.readTouch() == false)
               {
                  p_touch_past = false;
                  Keyboard.release('4');
                  delay(10);
               }
            }
      
            //2 Touch Macros
            
            //3 Touch Macros
            
            //4 Touch Macro
         }
         else if(sub_config == 3)
         {
            // ================================================================
            // ===                        IMU SENSOR                        ===
            // ================================================================
            if(neg_x_past == true)
               roll_imu = -420;
            if(pos_x_past == true)
               roll_imu = 420;
            if(neg_y_past == true)
               pitch_imu = -69;
            if(pos_y_past == true)
               pitch_imu = 69;
            
            if(roll_imu < (-1 * roll_min))
            {  
               if(neg_x_past == false)
               {
                  neg_x_past = true;
                  Keyboard.press('A');
               }
               else if(neg_x_past == true && real_roll_imu >= (-1 * roll_min))
               {
                  neg_x_past = false;
                  Keyboard.release('A');
               }
            }
            else if(roll_imu > roll_min)
            {  
               if(pos_x_past == false)
               {
                  pos_x_past = true;
                  Keyboard.press('D');
               }
               else if(pos_x_past == true && real_roll_imu <= roll_min)
               {
                  pos_x_past = false;
                  Keyboard.release('D');
               }
            }
            
            if(pitch_imu < (-1 * pitch_min))
            {  
               if(neg_y_past == false)
               {
                  neg_y_past = true;
                  Keyboard.press('S');
               }
               else if(neg_y_past == true && real_pitch_imu >= (-1 * pitch_min))
               {
                  neg_y_past = false;
                  Keyboard.release('S');
               }
            }
            else if(pitch_imu > pitch_min)
            {  
               if(pos_y_past == false)
               {
                  pos_y_past = true;
                  Keyboard.press('W');
               }
               else if(pos_y_past == true && real_pitch_imu <= pitch_min)
               {
                  pos_y_past = false;
                  Keyboard.release('W');
               }
            }
         }
      }
      
      // ================================================================
      // ===                     CONFIGURATION 3                      ===
      // ================================================================
      else if(curr_config_state == 3)
      {
         // ================================================================
         // ===                        FLEX SENSOR                       ===
         // ================================================================
         //1 Flex Macros
   
         //2 Flex Macros
         
         //3 Flex Macros
     
         //4 Flex Macros
         
         //5 Flex Macros
         
         // ================================================================
         // ===                       TOUCH SENSOR                       ===
         // ================================================================
         //1 Touch Macros
   
         //2 Touch Macros
         if(i_touch.getTouch() == true && m_touch.getTouch() == true && r_touch.getTouch() == false && p_touch.getTouch() == false)
         {
            if(i_touch_past == false && m_touch_past == false)
            {
               i_touch_past = true;
               m_touch_past = true;
               delay(15);
               Keyboard.press('E');
            }
            else if((i_touch_past == true && i_touch.readTouch() == false) || (m_touch_past == true && m_touch.readTouch() == false))
            {
               i_touch_past = false;
               m_touch_past = false;
               Keyboard.release('E');
               delay(10);
            }
         }
         else if(i_touch.getTouch() == false && m_touch.getTouch() == true && r_touch.getTouch() == true && p_touch.getTouch() == false)
         {
             if(m_touch_past == false && r_touch_past == false)
            {
               m_touch_past = true;
               r_touch_past = true;
               delay(15);
               Keyboard.press('C');
            }
            else if((m_touch_past == true && m_touch.readTouch() == false) || (r_touch_past == true && r_touch.readTouch() == false))
            {
               m_touch_past = false;
               r_touch_past = false;
               Keyboard.release('C');
               delay(10);
            }
         }
         else if(i_touch.getTouch() == false && m_touch.getTouch() == false && r_touch.getTouch() == true && p_touch.getTouch() == true)
         {
             if(r_touch_past == false && p_touch_past == false)
            {
               r_touch_past = true;
               p_touch_past = true;
               delay(15);
               Keyboard.press(' ');
            }
            else if((r_touch_past == true && r_touch.readTouch() == false) || (p_touch_past == true && p_touch.readTouch() == false))
            {
               r_touch_past = false;
               p_touch_past = false;
               Keyboard.release(' ');
               delay(10);
            }
         }
      
         //3 Touch Macros
         else if(i_touch.getTouch() == true && m_touch.getTouch() == true && r_touch.getTouch() == true && p_touch.getTouch() == false)
         {
            if(i_touch_past == false && m_touch_past == false && r_touch_past == false)
            {
               i_touch_past = true;
               m_touch_past = true;
               r_touch_past = true;
               delay(15);
               Keyboard.press('9');
            }
            else if((i_touch_past == true && i_touch.readTouch() == false) || (m_touch_past == true && m_touch.readTouch() == false) || (r_touch_past == true && r_touch.readTouch() == false))
            {
               i_touch_past = false;
               m_touch_past = false;
               r_touch_past = false;
               Keyboard.release('9');
               delay(10);
            }
         }
         else if(i_touch.getTouch() == false && m_touch.getTouch() == true && r_touch.getTouch() == true && p_touch.getTouch() == true)
         {
            if(m_touch_past == false && r_touch_past == false && p_touch_past == false)
            {
               m_touch_past = true;
               r_touch_past = true;
               p_touch_past = true;
               delay(15);
               Keyboard.press('2');
            }
            else if((m_touch_past == true && m_touch.readTouch() == false) || (r_touch_past == true && r_touch.readTouch() == false) || (p_touch_past == true && p_touch.readTouch() == false))
            {
               m_touch_past = false;
               r_touch_past = false;
               p_touch_past = false;
               Keyboard.release('2');
               delay(10);
            }
         }
      
         //4 Touch Macro
         
         // ================================================================
         // ===                        IMU SENSOR                        ===
         // ================================================================
   
      }
   }
   
   //If the switch is turned "off", make sure all the values get reset
   else
   {
      t_flex.reset();
      i_flex.reset();
      m_flex.reset();
      r_flex.reset();
      p_flex.reset();
      i_touch.reset();
      m_touch.reset();
      r_touch.reset();
      p_touch.reset();
      yaw_imu = 0;
      pitch_imu = 0;
      roll_imu = 0;
      x_motion = 0;
      y_motion = 0;
      z_motion = 0;
      t_flex_past = false;
      i_flex_past = false;
      m_flex_past = false;
      r_flex_past = false;
      p_flex_past = false;
      all_false_flex = false;
      scroll_status = false;
      i_touch_past = false;
      m_touch_past = false;
      r_touch_past = false;
      p_touch_past = false;
      pos_x_past = false;
      neg_x_past = false;
      pos_y_past = false;
      neg_y_past = false;
      if(other_fingers == true)
      {
         Keyboard.release('8');
         other_fingers = false;
      }
   }

   // ================================================================
   // ===                         DEBUGGING                        ===
   // ================================================================
   //Print all values for analysis/debugging 
   Serial.print(t_flex.getFlex());
   Serial.print("\t");
   Serial.print(i_flex.getFlex());
   Serial.print("\t");
   Serial.print(m_flex.getFlex());
   Serial.print("\t");
   Serial.print(r_flex.getFlex());
   Serial.print("\t");
   Serial.print(p_flex.getFlex());
   Serial.print("\t");
   Serial.print(i_touch.getTouch());
   Serial.print("\t");
   Serial.print(m_touch.getTouch());
   Serial.print("\t");
   Serial.print(r_touch.getTouch());
   Serial.print("\t");
   Serial.print(p_touch.getTouch());
   Serial.print("\t");
   Serial.print(yaw_imu);
   Serial.print("\t");
   Serial.print(pitch_imu);
   Serial.print("\t");
   Serial.print(roll_imu);
   Serial.print("\t");
   Serial.print(x_motion);
   Serial.print("\t");
   Serial.print(y_motion);
   Serial.print("\t");
   Serial.println(z_motion);
}
