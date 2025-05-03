/*
Connections between the ESP32 and sensors described below:

----------Magnetic Encoder (AS5600)----------
        1     2     3
VCC -> 3V3   3V3   3V3
OUT ->
GND -> GND   GND   GND
DIR -> 
SCL -> SC7   SC5   SC3 (CONNECTED TO I2C MULTIPLEXER)
SDA -> SD7   SD5   SD3 (CONNECTED TO I2C MULTIPLEXER)
GPO -> 3V3   3V3   3V3

----------I2C 8-Channel Multiplexer (PCA9548A)----------
VIN -> 3V3
GND -> GND
SDA -> D21
SCL -> D22
RST ->
A0  ->
A1  ->
A2  ->
SD0 ->
SC0 ->
SD1 ->
SC1 ->
SD2 ->
SC2 ->
SD3 -> SDA (CONNECTED TO 3RD ENCODER)
SC3 -> SCL (CONNECTED TO 3RD ENCODER)
SD4 ->
SC4 ->
SD5 -> SDA (CONNECTED TO 2ND ENCODER)
SC5 -> SCL (CONNECTED TO 2ND ENCODER)
SD6 ->
SC6 ->
SD7 -> SDA (CONNECTED TO 1ST ENCODER)
SC7 -> SCL (CONNECTED TO 1ST ENCODER)

----------6-axis IMU (MPU6050)----------
VCC -> 3V3
GND -> GND
SCL -> D22
SDA -> D21
XDA -> 
XCL -> 
ADO -> 
INT -> 

----------Motor Driver (Dual TB6612FNG 1A)----------
          1     2     3
PWMA  -> D5    D32   D26
AIN2  -> D19   D33   D27
AIN1  -> D18   D25   D14
STBY  -> 3V3   3V3   3V3
BIN1  -> 
BIN2  -> 
PWMB  -> 
GND   -> 
VM    -> +VE   +VE   +VE (FROM POWER SUPPLY OR BATTERY)
VCC   -> 3V3   3V3   3V3
GND   -> GND   GND   GND (ALSO CONNECTED TO POWER SUPPLY GND)
AO1   -> (CONNECTED TO RESPECTIVE MOTOR RED TERMINAL)
AO2   -> (CONNECTED TO RESPECTIVE MOTOR WHITE TERMINAL)
BO2   -> 
BO1   -> 
GND   -> 

----------DC Motor (GA25-370 12V 500RPM)----------
RED   -> A01 (CONNECTED TO RESPECTIVE MOTOR DRIVER)
WHITE -> AO2 (CONNECTED TO RESPECTIVE MOTOR DRIVER)
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "SensorFusion.h"
#include <AS5600.h>
#include "Tachometer.h"


// Define pitch and roll setpoints of different balancing orientations
// The first three are the edge-balancing values and the last one is the corner-balancing value
const double ROLL[4] = {-48.5, 1000, 1000, 1000}; // [deg]
const double PITCH[4] = {0, 1000, 1000, 1000}; // [deg]
double roll_setpoint = -1;
double pitch_setpoint = -1;
bool block_roll[3] = {false, true, true};


// Define Motor Control Pins for the 3 Motor Drivers
const int PWMA[3] = {5, 32, 26};
const int AIN2[3] = {19, 33, 27};
const int AIN1[3] = {18, 25, 14};
// PWM frequency and resolution for motor driver
const int FREQ = 8000;
const int RESOLUTION = 12;  // 12-bit (0-4095)
const int MAX_PWM = (1 << RESOLUTION);
// Motor Driver Function
void moveMotor(int speed, bool forward, int motor) {
  digitalWrite(AIN1[motor], forward ? HIGH : LOW);
  digitalWrite(AIN2[motor], forward ? LOW : HIGH);
  ledcWrite(PWMA[motor], speed);
}



// AS5600 sensor object
AS5600 as5600;
// AS5600 connection to PCA9548A
const double encoder[3] = {7, 5, 3};


// MPU6050 sensor object
Adafruit_MPU6050 imu;
// To store Accelerometer [m/s^2] and Gyroscope [rad/s] data
sensors_event_t accel, gyro;
Adafruit_Sensor *imu_accel, *imu_gyro;
// Sensor Fusion object
SF fusion;
float pitch, roll;
float deltat;


// Settings
const double ANGLE_LIMIT = 4; // The recovery angle [deg]
const double ANGLE_FIXRATE = 1.0; // variate target angle [deg/s]
const double ANGLE_FIXRATE_2 = 0.1; // reduce continuous rotation
const double Kp = 0.4; // PID proportional factor
const double Ki = 0.05; // PID integral factor
const double Kd = 0.001; // PID derivative factor
const double MOTOR_R = 12.1; // motor resistance [Ohm]
const double MOTOR_Ke = 0.272; // motor back EMF constant [Vs/rad]
const double SUPPLY_VOLTAGE = 12; // battery box voltage [V]
const double WHEEL_AV_INTERVAL = 25; // wheel angular velocity calculation interval [ms]
const int SLEEP_TIME = 1; // main loop sleep [ms]
const double K_d[3] = {-768.5775, -93.0741,  -1.2917}; // Gain matrix for LQR Control


unsigned long nowTime;
unsigned long prevMainTime;


// PID variables
double integral = 0;
double derivative;
double error;
double previous_error = 0;
double PID_output;
double current;
double voltage;
double motorCtrl;


// LQR variables
double LQR_output;
double theta_dot;
const double DegToRad = PI/180.0;


// Tachometer variables
double cur_pos;
unsigned long prevTachoTime;
double tacho_dt;
double omega[3] = {0, 0, 0};
Tachometer tac[3];


void PCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70); // PCA9548A address is 0x70
  Wire.write(1 << bus); // Send byte to select bus
  Wire.endTransmission();
}


void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Default I2C pins on ESP32
  delay(100); // Allow bus to stabalize



  // Initialize IMU sensor object
  if (!imu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
      delay(10);
  }
  Serial.println("MPU6050 Found!");
  
  imu_accel = imu.getAccelerometerSensor();
  // imu_accel->printSensorDetails();
  imu_gyro = imu.getGyroSensor();
  // imu_gyro->printSensorDetails();



  // Configure motor control pins as outputs
  for (int i = 0; i < 3; ++i){
    pinMode(AIN1[i], OUTPUT);
    pinMode(AIN2[i], OUTPUT);
    pinMode(PWMA[i], OUTPUT);
    // Setup PWM for Motor Driver
    ledcAttach(PWMA[i], FREQ, RESOLUTION);
  }



  // Initialize AS5600 sensor object
  for (int i = 0; i < 3; ++i){
    PCA9548A(encoder[i]); // Select channel on PCA9548A

    if (!as5600.begin()) {
      Serial.printf("Failed to find AS5600 chip #%d\n", i);
      while (1)
        delay(1000);
    }
    Serial.printf("AS5600 #%d Found!\n", i);

    tac[i].setPos(as5600.rawAngle());
  }

  prevMainTime = prevTachoTime = millis();
  delay(1000);
}


void loop() {
  nowTime = millis();



  // Tachometer ----------------------------------------------------------------------------------------------------
  if (nowTime - prevTachoTime >= WHEEL_AV_INTERVAL){
    tacho_dt = (nowTime - prevTachoTime) / 1000.0; // Convert to seconds
    prevTachoTime = nowTime;

    for (int i = 0; i < 3; ++i){
      PCA9548A(encoder[i]); // Select channel on PCA9548A

      cur_pos = as5600.rawAngle();
      omega[i] = tac[i].calcSpeed(cur_pos, tacho_dt);
    }
  }



  if (nowTime - prevMainTime >= SLEEP_TIME){
    prevMainTime = nowTime;

    // IMU for attitude estimation -----------------------------------------------------------------------------------
    imu_accel->getEvent(&accel);
    imu_gyro->getEvent(&gyro);

    deltat = fusion.deltatUpdate();
    // Mahony is suggested if the magnetometer data isn't available and for faster speed
    fusion.MadgwickUpdate(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
                        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                        deltat);
    // fusion.MahonyUpdate(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, 
    //                     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
    //                     deltat);
    roll = fusion.getRoll();
    pitch = fusion.getPitch();
    Serial.printf("%f,", roll);
    Serial.printf("%f,", pitch);



    // Control ----------------------------------------------------------------------------------------------------
    bool rest = true;

    for (int i = 0; i < 4; ++i)
      if (abs(roll - ROLL[i]) <= ANGLE_LIMIT && abs(pitch - PITCH[i]) <= ANGLE_LIMIT){
        rest = false;

        if (i <= 3){ // Edge-balancing
          if (!block_roll[i]){
            if (roll_setpoint == -1)
              roll_setpoint = ROLL[i];
            pitch_setpoint = -1;

            error = roll_setpoint - roll;

            // variate target angle, for uneven terrain and robustness
            if (error > 0)
                roll_setpoint += ANGLE_FIXRATE * deltat;
            else
                roll_setpoint -= ANGLE_FIXRATE * deltat;

            // reduce continuous rotation
            roll_setpoint -= ANGLE_FIXRATE_2 * omega[i] * deltat;
          }
          else{
            if (pitch_setpoint == -1)
              pitch_setpoint = PITCH[i];
            roll_setpoint = -1;

            error = pitch_setpoint - pitch;

            // variate target angle, for uneven terrain and robustness
            if (error > 0)
                pitch_setpoint += ANGLE_FIXRATE * deltat;
            else
                pitch_setpoint -= ANGLE_FIXRATE * deltat;

            // reduce continuous rotation
            pitch_setpoint -= ANGLE_FIXRATE_2 * omega[i] * deltat;
          }



          
          // PID Controller
          derivative = (error - previous_error) / deltat; // Backward Differentiation
          integral += (error + previous_error) * deltat / 2.0; // Trapezoidal Rule for Integration
          PID_output = Kp * error + Ki * integral + Kd * derivative;
          previous_error = error;

          // compensate for motor back EMF voltage
          current = PID_output;
          voltage = MOTOR_R * current + MOTOR_Ke * omega[i];

          // convert voltage to pwm duty cycle
          motorCtrl = voltage / SUPPLY_VOLTAGE * MAX_PWM;




          // // LQR Controller
          // derivative = (error - previous_error) / deltat; // Backward Differentiation
          // previous_error = error;
          // error *= DegToRad;
          // derivative *= DegToRad;
          // omega[i] *= 2*PI;
          // LQR_output = -(K_d[0]*error + K_d[1]*derivative + K_d[2]*omega[i]);

          // // convert LQR output to pwm duty cycle
          // motorCtrl = LQR_output / SUPPLY_VOLTAGE * MAX_PWM;

          
          
          // Determine speed and motor direction
          bool forward = (motorCtrl < 0);
          int speed = min(abs(int(motorCtrl)), MAX_PWM); // Limit speed to max PWM value

          moveMotor(speed, forward, i);
        }

        else{ // Corner-balancing
          // Work in progress
        }

        break;
      }

    if (rest){
      roll_setpoint = pitch_setpoint = -1;
      integral = 0;
      error = 0;
      previous_error = 0;
      for (int i = 0; i < 3; ++i)
        moveMotor(0, 0, i);
    }
    
    
    
    // ----------------------------------------------------------------------------------------------------
    // Serial.printf("%lf,%lf,%lf,", omega[0], omega[1], omega[2]);
    Serial.println("");
  }
}