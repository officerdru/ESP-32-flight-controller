

// Libraries to be included 


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

Adafruit_MPU6050 mpu;
WiFiUDP UDP;

// ---------------------------------------------------------------------------------------------------------

// debugging variables

unsigned long debugt1, debugt2, currenttime, prevtime = 0;

// ---------------------------------------------------------------------------------------------------------


// Filter Variables

float thr_des=0.5 , roll_des = 0, yaw_des = 0, pitch_des = 0;
double gyro_x_err, gyro_y_err, gyro_z_err, pitch_err, roll_err;
double gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z;
double angle_roll_gyro,angle_pitch_gyro,angle_yaw_gyro,angle_pitch_accel,angle_roll_accel;
double pitch_angle,roll_angle,yaw_angle;

double dt, alpha= 0.96; // alpha needs to be adjusted based on sampling rate

bool already_started = false;
bool calibrated=false;

// -----------------------------------------------------------------------------------------------------------

// PID Parameters

float Kp_roll_angle =3;    //Roll P-gain - angle mode 
float Ki_roll_angle =0 ;    //Roll I-gain - angle mode
float Kd_roll_angle =0 ;   //Roll D-gain - angle mode

float Kp_pitch_angle = 3;   //Pitch P-gain - angle mode
float Ki_pitch_angle =0;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0;  //Pitch D-gain - angle mode

float Kp_yaw = 0;           //Yaw P-gain
float Ki_yaw = 0;          //Yaw I-gain
float Kd_yaw = 0;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

float error_pitch, error_roll, error_yaw, error_yaw_prev;

float integral_pitch, integral_yaw, integral_roll;
float integral_pitch_prev, integral_yaw_prev, integral_roll_prev;
float i_limit = 25/7.5;

float derivative_pitch, derivative_roll, derivative_yaw;

float yaw_PID, roll_PID, pitch_PID;


// Tuning parameters

float kp_roll_angle_tuning_scaler = 0.01;
float ki_roll_angle_tuning_scaler = 0.01;
float kd_roll_angle_tuning_scaler = 0.001;

float kp_pitch_angle_tuning_scaler = 0.01;
float ki_pitch_angle_tuning_scaler = 0.01;
float kd_pitch_angle_tuning_scaler = 0.001;

float kp_yaw_tuning_scaler = 0.01;
float ki_yaw_tuning_scaler = 0.001;
float kd_yaw_tuning_scaler = 0.00001;

// -----------------------------------------------------------------------------------------------------------------------

// PWM parameters

float mfl_norm, mbl_norm, mfr_norm, mbr_norm;
int mfl, mbl, mfr, mbr;

const int PWMFreq = 25000; /* 50hz for esc*/
const int PWMChannel_fl = 0, PWMChannel_fr = 1, PWMChannel_bl = 2, PWMChannel_br = 4;
const int PWMResolution = 8;
const int max_duty_cycle = (int)(pow(2, PWMResolution) - 1);
const int m_fl_pin = 13, m_bl_pin = 27, m_fr_pin = 32, m_br_pin = 33; 


// ------------------------------------------------------------------------------------------------------------------------

// Communication Parameters

const char *soft_ap_ssid = "MyESP32AP";
const char *soft_ap_password = "123456789";
unsigned int port = 50000;  // local port 
char recieved_packet[5];
int cmd; //to read value for calibration

// ------------------------------------------------------------------------------------------------------------------------

 void calibrate()
 // This function would calibrate the ESC's 
  {
   while (calibrated==false) {
    
           Serial.println("calibrating...");    

       while(Serial.available()==0){

       }
        
     cmd=Serial.parseInt();
     switch(cmd){
         case 1 :ledcWrite(PWMChannel_fl,max_duty_cycle/20);
                 ledcWrite(PWMChannel_fr,max_duty_cycle/20);
                 ledcWrite(PWMChannel_bl,max_duty_cycle/20);
                 ledcWrite(PWMChannel_br,max_duty_cycle/20);
                 Serial.println("At minimum throttle");
                 break;
         case 2 :ledcWrite(PWMChannel_fl,max_duty_cycle/10);
                 ledcWrite(PWMChannel_fr,max_duty_cycle/10);
                 ledcWrite(PWMChannel_bl,max_duty_cycle/10);
                 ledcWrite(PWMChannel_br,max_duty_cycle/10);
                 Serial.println("At maximum throttle");
                 break;
         case 3 :calibrated=true;
               break;
       }

   }
 }

 void setup() {
   Serial.begin(115200);

 WiFi.softAP(soft_ap_ssid, soft_ap_password);
 UDP.begin(port);

  // Try to initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  Wire.setClock(1000000);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  for (int j = 1; j<=200; j+=1){
    gyro_x_err += (g.gyro.x);

    gyro_y_err += (g.gyro.y);

    gyro_z_err += (g.gyro.z);
    
    delay(10);
  }
  // compenstating for calibration errors

  gyro_x_err/=200.0;
  gyro_y_err/=200.0;
  gyro_z_err/=200.0;

  pitch_err=atan2(double(-acc_x),double(sqrt((pow(acc_z,2)+pow(acc_y,2)))));
  roll_err=atan2(double(acc_y),double(sqrt((pow(acc_z,2)+pow(acc_x,2)))));


  // setup pwm pins
  ledcSetup(PWMChannel_fl, PWMFreq, PWMResolution);
  ledcSetup(PWMChannel_fr, PWMFreq, PWMResolution);
  ledcSetup(PWMChannel_bl, PWMFreq, PWMResolution);
  ledcSetup(PWMChannel_br, PWMFreq, PWMResolution);

  ledcAttachPin(m_fl_pin, PWMChannel_fl);
  ledcAttachPin(m_fr_pin, PWMChannel_fr);
  ledcAttachPin(m_bl_pin, PWMChannel_bl);
  ledcAttachPin(m_br_pin, PWMChannel_br);

  delay(2000);


  // calibrate();  //for calibration, will only run once
}


void loop() {
  //calibrate();  //for calibration, will only run once  
  /* Get new sensor events with the readings */
  debugt1=micros();  
  sensors_event_t a, g, temp;  
  mpu.getEvent(&a, &g, &temp);



  /* Print out the values */
  acc_x = (a.acceleration.x);

  acc_y = (a.acceleration.y);

  acc_z = (a.acceleration.z);



  gyro_x = (g.gyro.x) ;

  gyro_y = (g.gyro.y) ;

  gyro_z = (g.gyro.z) ;


  // After reading gyro,acc values
  //gyro_x-=Gyro_raw_error_x;
  //gyro_y-=Gyro_raw_error_y;
  //gyro_z-=Gyro_raw_error_z;
  //acc_x-=Acc_raw_error_x;
  //acc_y-=Acc_raw_error_y;
  //acc_z-=Acc_raw_error_z;
  //Calculating angles, roll-x,pitch-y,yaw-z
  prevtime=currenttime;
  currenttime=micros();

  dt=double ((currenttime-prevtime))/1000000;

  angle_roll_gyro=roll_angle+gyro_x*dt;
  angle_pitch_gyro=pitch_angle+gyro_y*dt;
  angle_yaw_gyro=yaw_angle+gyro_z*dt;
  angle_pitch_accel=atan2(double(-acc_x),double(sqrt((pow(acc_z,2)+pow(acc_y,2)))));
  angle_roll_accel=atan2(double(acc_y),double(sqrt((pow(acc_z,2)+pow(acc_x,2)))));
  if(already_started)
  {
  pitch_angle=alpha*angle_pitch_gyro+(1-alpha)*angle_pitch_accel;
  roll_angle=alpha*angle_roll_gyro+(1-alpha)*angle_roll_accel;
  yaw_angle=angle_yaw_gyro;
  }
  else 
  {
  pitch_angle=angle_pitch_accel;
  roll_angle=angle_roll_accel;
  yaw_angle=0;
  already_started= true;
  }
  controlANGLE();
  mixer();
  scaler();
  command_motors();
  get_input();
  constrain_des_states();
  Serial.println(gyro_x);
  Serial.print(" | ");
  Serial.print(gyro_y);
   Serial.print(" | ");
  Serial.print(gyro_z);
   Serial.print(" | ");
  Serial.print(acc_x);
   Serial.print(" | ");
  Serial.print(acc_y);
   Serial.print(" | ");
  Serial.print(acc_z);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implemented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  //Roll

  error_roll = roll_des - roll_angle;
  integral_roll = integral_roll_prev + error_roll*dt;
  // if (thr_des < 0.1) {   //Don't let integrator build if throttle is too low
  //   integral_roll = 0;
  // }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = gyro_x;
  roll_PID = 0.1*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_angle;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  /*if (thr_des < 0.1) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }*/
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = gyro_y;
  pitch_PID = 0.1*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - gyro_z;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  /*if (thr_des < 0.1) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }*/
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = 0.1*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void mixer()
// The mixer assigns a value to be used to generate the PWM signal for each motor depending on the axis the drone has to rotate
 {
  mfl_norm = thr_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  mfr_norm = thr_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  mbr_norm = thr_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  mbl_norm = thr_des + pitch_PID + roll_PID - yaw_PID; //Back Left
}

void scaler()
// The scaler uses the values obtained from the mixer and sends a PWM signal taking in to consideration the maximum and minimum duty cycles
 {
  mfl = max_duty_cycle/2 + mfl_norm*max_duty_cycle/2;
  mbl = max_duty_cycle/2 + mbl_norm*max_duty_cycle/2;
  mfr = max_duty_cycle/2 + mfr_norm*max_duty_cycle/2;
  mbr = max_duty_cycle/2 + mbr_norm*max_duty_cycle/2;
}

void command_motors() {
  ledcWrite(PWMChannel_fl, mfl);
  ledcWrite(PWMChannel_fr, mfr);
  ledcWrite(PWMChannel_bl, mbl);
  ledcWrite(PWMChannel_br, mbr);  
}
void get_input()
//This function would recieve an integer over UDP, depending on the integer it will increase a value, decrease a value or change modes.
{
  int packetsize=UDP.parsePacket();
  if(packetsize){
    int len = UDP.read(recieved_packet,5);
    if(recieved_packet[0]==2){
       thr_des+=0.05;
    }
    else if(recieved_packet[0]==0){
         thr_des-=0.05;
    }
    
    if(recieved_packet[4]==2){
       if(recieved_packet[1]==2){
        Kp_pitch_angle+=kp_pitch_angle_tuning_scaler; 
       }
       else if(recieved_packet[1]==0){
         Kp_pitch_angle-=kp_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[2]==2){
         Kp_roll_angle+=kp_roll_angle_tuning_scaler; 
       }
       else if(recieved_packet[2]==0){
         Kp_roll_angle-=kp_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[3]==2){
         Kp_yaw+=kp_yaw_tuning_scaler; 
       }
       else if(recieved_packet[3]==0){
         Kp_yaw-=kp_yaw_tuning_scaler; 
       }
    }
    if(recieved_packet[4]==1){
       if(recieved_packet[1]==2){
        Ki_pitch_angle+=ki_pitch_angle_tuning_scaler; 
       }
       else if(recieved_packet[1]==0){
         Ki_pitch_angle-=ki_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[2]==2){
         Ki_roll_angle+=ki_roll_angle_tuning_scaler; 
       }
       else if(recieved_packet[2]==0){
         Ki_roll_angle-=ki_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[3]==2){
         Ki_yaw+=ki_yaw_tuning_scaler; 
       }
       else if(recieved_packet[3]==0){
         Ki_yaw-=ki_yaw_tuning_scaler; 
       }

    }
    if(recieved_packet[4]==0){
       if(recieved_packet[1]==2){
        Kd_pitch_angle+=kd_pitch_angle_tuning_scaler; 
       }
       else if(recieved_packet[1]==0){
         Kd_pitch_angle-=kd_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[2]==2){
         Kd_roll_angle+=kd_roll_angle_tuning_scaler; 
       }
       else if(recieved_packet[2]==0){
         Kd_roll_angle-=kd_pitch_angle_tuning_scaler; 
       }
       if(recieved_packet[3]==2){
         Kd_yaw+=kd_yaw_tuning_scaler; 
       }
       else if(recieved_packet[3]==0){
         Kd_yaw-=kd_yaw_tuning_scaler; 
       }

     }

  }
}

void constrain_des_states(){
  thr_des = constrain(thr_des,0,1);
  pitch_des = constrain(pitch_des,-1,1);
  roll_des = constrain(roll_des,-1,1);
  yaw_des = constrain(yaw_des,-1,1);
}

