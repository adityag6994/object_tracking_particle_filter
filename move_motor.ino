///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 


//                                             Soft Robotics Lab 
//                                           Quad Tilt Wing Project 
//                                          By Aihaitijiang Abudula
//                                                    WPI

//                                  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////Flight tilt angle 0 to 90 + Altitude hold   Final Flight Code//////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////







#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Servo.h> 


//////////////////////////////////////////////////Servo///////////////////////////////////////////////////////////////////////////////

Servo myservo_front;
Servo myservo_rear;

int degree_front;
int degree_rear;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// PID altitude
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean altHold, altRead;
float SonarPin=A1;
float pid_p_gain_alt = 0.6;//0.6
float pid_i_gain_alt = 0.8;//0.8
float pid_d_gain_alt = 9.0;//5
int pid_max_alt = 300; // 5m hight 
float pid_i_mem_alt , pid_alt_setpoint, pid_alt_input, pid_output_alt, pid_last_alt_d_error, pid_alt_ground;
int  pid_alt_throttle;

float cm;
float Inch;
float sensorValue;
float pid_error_temp_alt;

//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);
  pinMode(SonarPin,INPUT);

  //  Servo for tilt angle rear wing and front wing 
  myservo_front.attach(3);
  myservo_rear.attach(2);
}

void loop(){

  

//  Tilt_angle();
//  myservo_front.write(degree_front);  
//  myservo_rear.write(degree_rear); 

int i;
  for(i=60; i<=150; i++){
    myservo_rear.write(i); 
    delay(200);
  }
  delay(100);
  for(int j=i; j>=60; j--){
    myservo_rear.write(j); 
    delay(200);
  }
  delay(100);

Serial.print("\n");


 }




int Tilt_angle(){

 
  char param2 = Serial.read();  
  switch (param2)  {
  
   case '9':
         degree_front = 90-50;
         degree_rear = 170;
       break;
   case '8':
         degree_front = 105-50;
         degree_rear = 150;
       break;  

   case '7':
         degree_front = 120-50;
         degree_rear = 145;
       break;
   case '6':
         degree_front = 135-50;
         degree_rear = 110;
       break;     

   case '5':
         degree_front = 150-50;
         degree_rear = 90;
       break;
       
   case '4':
         degree_front = 165-50;
         degree_rear = 70;
       break;
       
   case '3':
         degree_front = 180-50;
         degree_rear = 50;
       break;     

   case '2':
         degree_front = 195-50;
         degree_rear = 30;
       break;
   case '1':
         degree_front = 195-50;
         degree_rear = 10;        

       
   default:
        Serial.print("");
       
       
       
  }  
}



