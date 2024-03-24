#include <SoftwareSerial.h>


SoftwareSerial OffboardSoftwareSerial(A0, A1); // RX, TX

#define L298_enA 9
#define L298_in1 4
#define L298_in2 2

#define L298_enB 10
#define L298_in3 13
#define L298_in4 12

#define RC_Ch1_Rotation 6
#define RC_Ch3_Throttle 3
#define RC_Ch5_Mode     5

int   leftSpeed, rightSpeed = 0.0;
float Linear_vel, Angula_vel = 0.0;
char  _RCMode;
int   _RCThrottle,_RCRotation = 0;
float maxMotorSpeed = 1.2;

float CalculatePWM(int Pin)
{
  double PulseHigh, PulseLow, DutyCycle;
  PulseHigh = pulseIn(Pin, HIGH, 100000UL);
  PulseLow  = pulseIn(Pin,  LOW, 100000UL);
  DutyCycle = (100.00 * PulseHigh) / (PulseHigh + PulseLow);
  return DutyCycle;
} 
void CalculateMotorSpeed(float linearVelocity, float angularVelocity, int& leftSpeed, int& rightSpeed)
{

  float robotRadius = 0.035;
  float maxMotorPWM =255.0;
   Serial.print(" linearVelocity : ");
   Serial.print(linearVelocity);
   Serial.print(" angularVelocity : ");
   Serial.println(angularVelocity);

  leftSpeed  = ((linearVelocity - (angularVelocity * robotRadius * 60.0) / 2.0)) * maxMotorPWM / maxMotorSpeed;
  rightSpeed = ((linearVelocity + (angularVelocity * robotRadius * 60.0) / 2.0)) * maxMotorPWM / maxMotorSpeed ;

  leftSpeed = (int) leftSpeed;
  rightSpeed = (int) rightSpeed;

   Serial.print(" leftSpeed : ");
   Serial.print(leftSpeed);
   Serial.print(" rightSpeed : ");
   Serial.println(rightSpeed);   

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
}

char ReadRCMode()
{
  int PWMRCModeRescaled;
  // Scale:   
  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  PWMRCModeRescaled = ((int)CalculatePWM(RC_Ch5_Mode) - 7.0) * (100.00 - 0.00) / (12.00 - 7.00) + 0.00;
  // Serial.print(CalculatePWM(RC_Ch1_Rotation));
  switch (PWMRCModeRescaled)
  {
    case  0:
      return 'P';
      break;
    case 60:
      return 'D';
      break;
    case 100:
      return 'A';
      break;
    default:
      return 'U';
      break;
  }
}
float  RCThrottle()
{
  float PWMThrottleRescaled;
  // Scale:   
  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  PWMThrottleRescaled = (CalculatePWM(RC_Ch3_Throttle) - 7.33) * (255.00 - (-255.00)) / (12.94 - 7.33) + (-255.00);
  return PWMThrottleRescaled;
}
float  RCRotation()
{
  float PWMRotationRescaled;
  PWMRotationRescaled = (CalculatePWM(RC_Ch1_Rotation) - 7.33) * (255.00 - (-255.00)) / (12.94 - 7.33) + (-255.00);
  return PWMRotationRescaled;
}

void ControlMode_Drive(float RCLinearVel, float RCAngularVel)
{
  float VelLinear,VelAngular;
    // Scale:   
  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
   VelLinear  = (_RCThrottle - (-255)) * (maxMotorSpeed - (-maxMotorSpeed)) / (255 - (-255)) + (-maxMotorSpeed);
   VelAngular = (_RCRotation - (-255)) * (maxMotorSpeed - (-maxMotorSpeed)) / (255 - (-255)) + (-maxMotorSpeed);
  CalculateMotorSpeed(VelLinear, VelAngular, leftSpeed, rightSpeed );
}
void ControlMode_Park()
{
  CalculateMotorSpeed(0.0, 0.0, leftSpeed, rightSpeed );
}
void ControlMode_OffBoard()
{
  if (OffboardSoftwareSerial.available()) 
  {
    //Read serial data from Odroid pyserial
    String data = OffboardSoftwareSerial.readStringUntil('\n');
    // Export linear and angular velocity
    int commaPosition = data.indexOf(',');
    Linear_vel = data.substring(0,commaPosition).toFloat();
    Angula_vel = data.substring(commaPosition+1).toFloat();

    // calculate pwm of motors
    if (Linear_vel < maxMotorSpeed && Angula_vel < maxMotorSpeed){
      CalculateMotorSpeed(Linear_vel, Angula_vel, leftSpeed, rightSpeed);
    }
  }
  else
  {
    Serial.print("      ... No OffBoard Data Received ...      \n");
  }
}

void DriveMotors()
{
    // motor A(left) enable signal handeling
    if( leftSpeed > 0){
      digitalWrite(L298_in1, HIGH);
      digitalWrite(L298_in2, LOW);
    }
    else if( leftSpeed < 0 ){
      digitalWrite(L298_in1, LOW);
      digitalWrite(L298_in2, HIGH);
    }
    else{
      digitalWrite(L298_in1, LOW);
      digitalWrite(L298_in2, LOW);
    }

    // motor A(right) enable signal handeling
    if( rightSpeed > 0){
      digitalWrite(L298_in3, HIGH);
      digitalWrite(L298_in4, LOW);
    }
    else if( rightSpeed < 0 ){
      digitalWrite(L298_in3, LOW);
      digitalWrite(L298_in4, HIGH);
    }
    else{
      digitalWrite(L298_in3, LOW);
      digitalWrite(L298_in4, LOW);
    }
    analogWrite(L298_enA, abs(leftSpeed));
    analogWrite(L298_enB, abs(rightSpeed));
}
//float Linear_vel, float Angula_vel, float leftSpeed, float rightSpeed
void LoggingData( )
{
  Serial.print("  R/C Mode: " );
  Serial.print(_RCMode);
  Serial.print("\n");
  switch (_RCMode)
  {
    case 'P':
      Serial.print("   Park Mode DATA :) \n");
      break;
    case 'D':
      Serial.print("  Forward PWM is : ");
      Serial.print(_RCThrottle);
      Serial.print("  Rotation PWM is : ");
      Serial.println(_RCRotation);
      break;
    case 'A':
      Serial.print("   Linear Velocity: ");
      Serial.print(Linear_vel);
      Serial.print("\n");
      Serial.print("   Angular velocity: ");
      Serial.print(Angula_vel);
      Serial.print("\n");
      break;
    default:
      break;
      //Serial.print(" Mode Not Vaild ");
  }
  Serial.print("   MotorSpeedA: ");
  Serial.print(leftSpeed);
  Serial.print("\n");  
  Serial.print("   MotorSpeedB: ");
  Serial.println(rightSpeed);
  Serial.print("\n");
  delay(200);
}

///////////////////////////////////
void setup() 
{
  Serial.begin(9600);
  OffboardSoftwareSerial.begin(9600);
  pinMode(RC_Ch1_Rotation,     INPUT_PULLUP);
  pinMode(RC_Ch3_Throttle, INPUT_PULLUP);
  pinMode(RC_Ch5_Mode,     INPUT_PULLUP);
}

void loop() 
{ // run over and over
  
  _RCMode = ReadRCMode();
  switch (_RCMode)
  {
    case 'P':
      CalculateMotorSpeed(0, 0, leftSpeed, rightSpeed);
      ControlMode_Park();
      break;
    case 'D':
      _RCThrottle = RCThrottle();
      _RCRotation = RCRotation();
      ControlMode_Drive(_RCThrottle,_RCRotation);
      break;
    case 'A':
      // Read Data From Offboard Computer

      ControlMode_OffBoard();
      break;
    default:
      Serial.print(" Mode Not Vaild ");
      break;
  }
  DriveMotors();
  LoggingData( );

  
/*  
    ///////////////////////////////////////////////////////////////////////
    // motor A(left) enable signal handeling
    if( leftSpeed > 0){
      digitalWrite(L298_in1, HIGH);
      digitalWrite(L298_in2, LOW);
    }
    else if( leftSpeed < 0 ){
      digitalWrite(L298_in1, LOW);
      digitalWrite(L298_in2, HIGH);
    }
    else{
      digitalWrite(L298_in1, LOW);
      digitalWrite(L298_in2, LOW);
    }

    // motor A(right) enable signal handeling
    if( rightSpeed > 0){
      digitalWrite(L298_in3, HIGH);
      digitalWrite(L298_in4, LOW);
    }
    else if( rightSpeed < 0 ){
      digitalWrite(L298_in3, LOW);
      digitalWrite(L298_in4, HIGH);
    }
    else{
      digitalWrite(L298_in3, LOW);
      digitalWrite(L298_in4, LOW);
    }
  ///////////////////////////////////////////////////////////////////////

  analogWrite(L298_enA, abs(leftSpeed));
  analogWrite(L298_enB, abs(rightSpeed));
  
  ///////////////////////////////////////////////////////////////////////
*/


}
