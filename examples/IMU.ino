#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <Wire.h>

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

const int pitch_max = 90;
const int pitch_min = 10;
const int roll_max = 90;
const int roll_min = 10;
const int mouse_min = 1;
const int mouse_max = 40;
int move_dist;
float yaw_imu = 0;
float pitch_imu = 0;
float roll_imu = 0;
int x_motion = 0;
int y_motion = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

int axisMotion(float ypr_data, char axis) 
{
   move_dist = (int) ypr_data;

   if(axis == 'x')
   {
      //Map the x-axis value to be within +/- 85 max and +/- 15 min accounting for noise on threshold
      if(move_dist >= (roll_min * -1) && move_dist <= roll_min)
         return 0;
      if (move_dist > roll_max) 
         move_dist = roll_max;
      if (move_dist < (roll_max * -1)) 
         move_dist = roll_max * -1;
      
      //Map move_dist to a linear mapping of mouse movement values
      if(move_dist > roll_min)
         move_dist = constrain(map(move_dist, roll_min,  roll_max, mouse_min, mouse_max), mouse_min, mouse_max);
      else
         move_dist = constrain(map(move_dist, (roll_min * -1),  (roll_max * -1), (mouse_min * -1), (mouse_max * -1)), (mouse_min * -1), (mouse_max * -1));
      
      return move_dist;
   }
   else if(axis == 'y')
   {
      //Map the x-axis value to be within +/- 85 max and +/- 15 min accounting for noise on threshold
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
      
      return move_dist;
   }
}

void setup() {
   Wire.begin();
   Serial.begin(115200);
   
   Serial.println(F("Initializing I2C devices..."));
   mpu.initialize();
   
   //Verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
   
   //Ready the buffer
   while (Serial.available() && Serial.read());
   
   //Load and configure the DMP
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();
   
   //Supply accel & gyro offsets
   mpu.setXGyroOffset(220);
   mpu.setYGyroOffset(76);
   mpu.setZGyroOffset(-85);
   mpu.setZAccelOffset(1788);
   /*
   mpu.setXAccelOffset(-4679);
   mpu.setYAccelOffset(-121);
   mpu.setZAccelOffset(591);
   mpu.setXGyroOffset(-1430);
   mpu.setYGyroOffset(-48);
   mpu.setZGyroOffset(-12);
   */
   
   //Make sure everything worked (returns 0 if so)
   if (devStatus == 0) 
   {
      //Turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      
      //Enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino DUE pin 2)..."));
      attachInterrupt(2, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      
      //Set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      
      //Get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
   } 
   else 
   {
      // Error has occurred
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
   }
}

void loop() {
   if(!dmpReady)
      return;
      
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();
   
   //Get current FIFO count
   fifoCount = mpu.getFIFOCount();
   
   //Check for overflow
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
   {
      //Reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
   } 
   //Otherwise, check for DMP data ready interrupt
   else if (mpuIntStatus & 0x02) 
   {
      //Wait for correct available data length
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
   
      //Read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
   
      //Track FIFO count here in case there is > 1 packet available
      //(this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
   
      //Convert values to ypr format
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      if(gravity.z>0) //range from 0 to 90 and from 0 to -90
        ypr[1] = atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
      else if(gravity.z<0) //to get range from 90 to 180 and ftom -90 to -180
        ypr[1] =  - (PI)-atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));     
      
      //Remapping Roll
      if(gravity.z>0) 
        ypr[2] = atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));
      else if(gravity.z<0)
        ypr[2] = -PI - atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));
      yaw_imu = ypr[0] * 180/M_PI;
      pitch_imu = ypr[1] * 180/M_PI;
      roll_imu = ypr[2] * 180/M_PI;
   }

   x_motion = axisMotion(roll_imu, 'x');
   y_motion = axisMotion(pitch_imu, 'y');
   Mouse.move(x_motion, y_motion, 0);

   Serial.print(yaw_imu);
   Serial.print("\t");
   Serial.print(pitch_imu);
   Serial.print("\t");
   Serial.print(roll_imu);
   Serial.print("\t");
   Serial.print(x_motion);
   Serial.print("\t");
   Serial.println(y_motion);
}
