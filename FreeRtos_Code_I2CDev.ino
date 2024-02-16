
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Wire.h"
#include "PIDCont.h"
#include "Config.h"
#include <WiFi.h>
#define LED_BUILTIN 33

//Uncomment only one full scale gyro range (deg/sec)
// #define GYRO_250DPS //Default
#define GYRO_500DPS
// #define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
// #define ACCEL_4G
// #define ACCEL_8G
//#define ACCEL_16G
#include "MPU6050.h"
MPU6050 mpu6050;

#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

SemaphoreHandle_t xDataMutex = NULL;  // Create a mutex object

SemaphoreHandle_t xMotorMutex = NULL;  // Create a mutex object

TaskHandle_t xDataAcquisition;
TaskHandle_t xActuation;
TickType_t xLastWakeTimeFilter,xLastWakeTimeIMU, xLastWakeTimeMotor;
//<--------------------------IMU Variables--------------------->>
typedef struct{

  float Axyz[3];
  float Gxyz[3];

}IMUData;


typedef struct{
  // GLOBALLY DECLARED, required for Mahony filter
  // vector to hold quaternion
  float roll;
  float pitch;
  float yaw;
}Attitude;

IMUData imuData;

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;

float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.12;  
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;


// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6050. They are not correct for other examples.
//The IMU code will NOT work well or at all if these are not correct
float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
// the code will work, but not as accurately, for an uncalibrated accelerometer. Use this line instead:
// float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz

float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

//<---------------------------Wifi Variables----------------------->>
int port = 5000;  //Port number
WiFiServer server(port);

typedef struct{
  float rx_roll;
  float rx_pitch;
  float rx_yaw;
  float rx_throttle;
}WifiData;

WifiData wifidata;
int tic=millis();

float setX = 0;
float setY = 0;
float setZ = 0;


//<--------------------------Mahony Variables--------------------->>


Attitude attitude;

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 20.0;
float Ki = 0.0;


//<-----------------------------PID Variables------------------------>>
PIDCont PIDroll,PIDpitch,PIDyaw,PIDangleX,PIDangleY;


void PID_init(){
    //                          Kp,        Ki,         Kd           Lval         Hval
  PIDroll.SetParams(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitch.SetParams(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyaw.SetParams(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
  PIDangleX.SetParams(ANGLEX_KP,ANGLEX_KI,ANGLEX_KD,ANGLEX_MIN,ANGLEX_MAX);
  PIDangleY.SetParams(ANGLEY_KP,ANGLEY_KI,ANGLEY_KD,ANGLEY_MIN,ANGLEY_MAX);

}

void motorInit(){
  pinMode(MOTOR0,OUTPUT); 
  pinMode(MOTOR1,OUTPUT);
  pinMode(MOTOR2,OUTPUT);
  pinMode(MOTOR3,OUTPUT);
  analogWrite(MOTOR0,MOTOR_ZERO_LEVEL);
  analogWrite(MOTOR1,MOTOR_ZERO_LEVEL);
  analogWrite(MOTOR2,MOTOR_ZERO_LEVEL);
  analogWrite(MOTOR3,MOTOR_ZERO_LEVEL);
}
//<<------------------------------------InitIMU-------------------------------

void IMUinit() {
  //DESCRIPTION: Initialize IMU

  Wire.begin();
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
  
  mpu6050.initialize();
  
  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }
  Serial.println("MPU6050 initialization successful");
  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}
//<<--------------------------------------------------------
void Wifi_init()
{
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password); //Connect to wifi
 
//   // Wait for connection  
//   //Serial.println("Connecting to Wifi");
//   while (WiFi.status() != WL_CONNECTED) {   
//     delay(500);
//    // Serial.print(".");
// //    delay(500);
//   }
//     server.begin();
}

void updateIMUData(float ax, float ay, float az, float gx, float gy,float gz)
{
  // IMU is mounted inverted, corresponding to 180 deg rotation about X thus quatities 
  // measured by IMU needs to be corrected for 180Deg Rotation, this y and Z measurements signs are inverted
  bool rotate180=false;
    xSemaphoreTake (xDataMutex, portMAX_DELAY);
    imuData.Axyz[0] = ax;
    if (rotate180){
    imuData.Axyz[1] = -ay;
    imuData.Axyz[2] = -az;
    }
    else{
      imuData.Axyz[1] = ay;
      imuData.Axyz[2] = az;
    }
    //apply offsets and scale factors from Magneto
    
    imuData.Gxyz[0] = gx;
    if (rotate180){
    imuData.Gxyz[1] = -gy;
    imuData.Gxyz[2] = -gz;
    }
    else{
      imuData.Gxyz[1] = gy;
      imuData.Gxyz[2] = gz;
    }

    xSemaphoreGive(xDataMutex); // Release the mutex
    // Serial.println("in Update IMU");
}



void getIMUData(float *ax,float *ay,float *az,float *gx,float *gy, float *gz)
{
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    *ax = imuData.Axyz[0];
    *ay = imuData.Axyz[1];
    *az = imuData.Axyz[2];
    *gx = imuData.Gxyz[0];
    *gy = imuData.Gxyz[1];
    *gz = imuData.Gxyz[2];
    xSemaphoreGive(xDataMutex); // Release the mutex
}


void updateAttitudeData(float roll,float pitch, float yaw)
{
    xSemaphoreTake(xMotorMutex, portMAX_DELAY);
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    xSemaphoreGive(xMotorMutex); // Release the mutex
}

void getAttitudeData(float *roll,float *pitch,float *yaw)
{   
    xSemaphoreTake(xMotorMutex, portMAX_DELAY);
    *roll = attitude.roll;
    *pitch = attitude.pitch;
    *yaw = attitude.yaw;
    xSemaphoreGive(xMotorMutex); // Release the mutex
}


void FetchIMUData(){
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature
  
  vTaskSuspend(xActuation);
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  vTaskResume(xActuation);

 //Accelerometer
  AccX = ax / ACCEL_SCALE_FACTOR; //G's
  AccY = ay / ACCEL_SCALE_FACTOR;
  AccZ = az / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = gx / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = gy / GYRO_SCALE_FACTOR;
  GyroZ = gz / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
  // if (DEBUG){
    Serial.print(AccX); Serial.print("\t");
    Serial.print(AccY); Serial.print("\t");
    Serial.print(AccZ); Serial.print("\t");
    // Serial.print(GyroX); Serial.print("\t");
    // Serial.print(GyroY); Serial.print("\t");
    // Serial.print(GyroZ); 
    Serial.print("\n");
  // }
  updateIMUData(AccX, AccY, AccZ, GyroX, GyroY,GyroZ);
  
}


  
  


// void FetchWifiData()
// {
//   WiFiClient client = server.available();
  
//   if (client) {
//     if(client.connected())
//     {
//       Serial.println("Client Connected");
//     }
//     while(client.connected()){
//       if ((millis()-tic)>15){
//       tic = millis();
//       readData(client);
//       // Serial.print('s');
//       // Serial.print(Vel);
//       // Serial.print('y');
//       // Serial.print(Yaw);
      
//       }
//       else{
//         //TODO state estimation
//         int a=1;
//       }
//         //>...................
        
      
//       //Send Data to connected client
// //      while(Serial.available()>0)
// //      {
// //        client.write(Serial.read());
// //      }
//     }
//     client.stop();
//     Serial.println("Client disconnected");    
//   }
// }





// Task function
void DataAcquisition(void *pvParameters) {
    while (1) {
      FetchIMUData();
      vTaskDelayUntil(&xLastWakeTimeIMU, pdMS_TO_TICKS(1));
      //FetchWifiData();
    }
}

void Actuation(void *pvParameters) {

    static float roll,pitch,yaw,throttle;
    throttle = 150;
    while (1) {
    {
    getAttitudeData(&roll,&pitch,&yaw);

    int m0_val = throttle + roll - pitch + yaw;
    int m1_val = throttle - roll - pitch - yaw;
    int m2_val = throttle - roll + pitch + yaw;
    int m3_val = throttle + roll + pitch - yaw;

    analogWrite(MOTOR0,m0_val);
    analogWrite(MOTOR1,m1_val);
    analogWrite(MOTOR2,m2_val);
    analogWrite(MOTOR3,m3_val);
    
    // Serial.print("\t");
    // Serial.print(m0_val);
    // Serial.print("\t");
    // Serial.print(m1_val);
    // Serial.print("\t");
    // Serial.print(m2_val);
    // Serial.print("\t");
    // Serial.print(m3_val);
    // Serial.println("\t");

  // #ifdef SAFE
  //   while(rxVal[5]<1100){
  //   };
  //   PIDroll.resetI();
  //   PIDpitch.resetI();
  //   PIDyaw.resetI();
  //   PIDangleX.resetI();
  //   PIDangleY.resetI();
  // #endif
  vTaskDelayUntil(&xLastWakeTimeMotor, pdMS_TO_TICKS(2));
  }
}
}




void setup() {
    // Create Task 1 pinned to Core 0
    Serial.begin(115200);
    delay(1);
    //Initialize IMU communication
    IMUinit();
    delay(1);
    PID_init();
    delay(1);
    motorInit();
    delay(1);
    // CaliberateIMU();
    xDataMutex = xSemaphoreCreateMutex();  // crete a mutex object
    xMotorMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(DataAcquisition, "DataAcquisition", 2048, NULL, 0, &xDataAcquisition, 0);
    xTaskCreatePinnedToCore(Actuation,"Actuation",2048,NULL,0,&xActuation,0);

}


void loop(){
  
  static float deltat,dt;
  static unsigned long now = 0, last = 0;
  static float ax,ay,az,gx,gy,gz;
  static float q[4] = {1.0, 0.0, 0.0, 0.0};
  getIMUData(&ax,&ay,&az,&gx,&gy,&gz);
  //Serial.println("in after IMU fetch");
  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  dt = deltat;
  last = now;
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;
  static float roll,pitch,yaw;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  
  if (tmp > 0.0)
    {
      // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
      recipNorm = 1.0 / sqrt(tmp);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Estimated direction of gravity in the body frame (factor of two divided out)
      vx = q[1] * q[3] - q[0] * q[2];  //to normalize these terms, multiply each by 2.0
      vy = q[0] * q[1] + q[2] * q[3];
      vz = q[0] * q[0] - 0.5f + q[3] * q[3];

      // Error is cross product between estimated and measured direction of gravity in body frame
      // (half the actual magnitude)
      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Compute and apply to gyro term the integral feedback, if enabled
      if (Ki > 0.0f) {
        ix += Ki * ex * deltat;  // integral error scaled by Ki
        iy += Ki * ey * deltat;
        iz += Ki * ez * deltat;
        gx += ix;  // apply integral feedback
        gy += iy;
        gz += iz;
      }

      // Apply proportional feedback to gyro term
      gx += Kp * ex;
      gy += Kp * ey;
      gz += Kp * ez;
    }
    
    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
    q[3] = q[3] * recipNorm;

    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    if (yaw < 0) yaw += 360.0; //compass circle
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // Serial.println(dt*1000);
    // Serial.print(roll);Serial.print("\t");
    // Serial.print(pitch);Serial.print("\t");
    // Serial.println(yaw);
    // Serial.print(gx);Serial.print("\t");
    // Serial.print(gy);Serial.print("\t");
    // Serial.println(gz);
    

    int PIDroll_val= (int)PIDroll.Compute(((float)setX-roll),-gy,dt);
    int PIDpitch_val= (int)PIDpitch.Compute(((float)setY-pitch),-gx,dt);
    int PIDyaw_val= (int)PIDyaw.Compute((float)setZ-gz,dt);
    
    // Serial.println(PIDroll_val);
    // Serial.println(PIDpitch_val);
    // Serial.println(PIDyaw_val);
  updateAttitudeData(PIDroll_val,PIDpitch_val,PIDyaw_val);
  vTaskDelayUntil(&xLastWakeTimeFilter, pdMS_TO_TICKS(1));
}