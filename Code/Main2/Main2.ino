#include <Wire.h>
#include <VarSpeedServo.h> 
#include <I2CEncoder.h>
#include <PID_v1.h>

//Servos
VarSpeedServo shoulderServo;
VarSpeedServo elbowServo;
VarSpeedServo handServo;
VarSpeedServo torsoServo;

////////////////////////////////

const int rightMotorPin = 8;
const int leftMotorPin = 9;
const int frontMotorPin = 10;
const int backMotorPin = 11;

VarSpeedServo leftMotor;
VarSpeedServo rightMotor;
VarSpeedServo frontMotor;
VarSpeedServo backMotor;

//Positions
double Left_Motor_Position;
double Right_Motor_Position;
double Front_Motor_Position;
double Back_Motor_Position;

double Forward_Position_GoTo;
double Right_Position_GoTo;
double Angle_Offset;

//Speed of Motors
double Left_Motor_Speed;
double Right_Motor_Speed;
double Front_Motor_Speed;
double Back_Motor_Speed;

int ultrasonic_front;
int ultrasonic_left;
int ultrasonic_back;
int ultrasonic_angle;

int ultrasonic_front_avg;
int ultrasonic_left_avg;
int ultrasonic_back_avg;
int ultrasonic_angle_avg;

//Encoder Positions
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_FrontMotor;
I2CEncoder encoder_BackMotor;

unsigned long currentMillis = 0; 
unsigned long previousMillis = 0;
const long interval = 1000; 

int adjustSpeed = 0;
int adjustPosition = 0;

//Specify the links and initial tuning parameters
double Kp=2, Ki=0.05, Kd=0.01;
PID leftPID(&Left_Motor_Position, &Left_Motor_Speed, &Forward_Position_GoTo, Kp, Ki, Kd, DIRECT);
PID rightPID(&Right_Motor_Position, &Right_Motor_Speed, &Forward_Position_GoTo, Kp, Ki, Kd, DIRECT);
PID frontPID(&Front_Motor_Position, &Front_Motor_Speed, &Right_Position_GoTo, Kp, Ki, Kd, DIRECT);
PID backPID(&Back_Motor_Position, &Back_Motor_Speed, &Right_Position_GoTo, Kp, Ki, Kd, DIRECT);

int cm[4];
const int US_MAX = 9999;

const int ci_Mode_Button = 12;
int  ui_Mode_Index = -1;
boolean bt_Time_Up = false;
boolean bt_Do_Once = false;
unsigned long bt_timer = 0;

///////////////////////////////


typedef enum ultrasonic { distance_front, distance_back, distance_up, distance_left, distance_angle };

//Force Sensor
const int forcePin = A0; 

// Hall Effect
const int ci_HallEffect_Data = A2;

// Line Sensor
const int ci_LineSensor_Data = A1;

const int shoulderPin = 3;    //numbers are wrong
const int elbowPin = 7;
const int handPin = 6;
const int torsoPin = 5;

void setup()
{
  Wire.begin();
  motorEncoderInit();
  //extraCore.setPinIOstate(pinNumber, OUTPUT);
  Serial.begin(9600);  
  Serial.println("Manager Ready.");

  updateScreen("loading...");
  //attaching servos
  shoulderServo.attach(shoulderPin, 500, 2500);
  elbowServo.attach(elbowPin, 500, 2500);
  handServo.attach(handPin, 500, 2500);
  torsoServo.attach(torsoPin, 500, 2500);

  leftMotor.attach(leftMotorPin, 500, 2500);
  rightMotor.attach(rightMotorPin, 500, 2500);
  frontMotor.attach(frontMotorPin, 500, 2500);
  backMotor.attach(backMotorPin, 500, 2500);
 
 
  Forward_Position_GoTo = 0;
  Right_Position_GoTo = 0;
  Left_Motor_Speed = 0;
  Angle_Offset = 0;
  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-500, 500);
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-500, 500);
  frontPID.SetMode(AUTOMATIC);
  frontPID.SetOutputLimits(-500, 500);
  backPID.SetMode(AUTOMATIC);
  backPID.SetOutputLimits(-500, 500);
  
  leftMotor.write(1500);
  rightMotor.write(1500);
  frontMotor.write(1500);
  backMotor.write(1500);
  
   
  // set up Hall Effect
  pinMode(ci_HallEffect_Data, INPUT);
  //digitalWrite(ci_HallEffect_Data, HIGH);
  startPosition();
   Serial.println("setup done.");
   for(int i = 0; i < 20; i++) {
     UpdateUltrasonics();
     delay(100);
   }
   moveArm(-1);
   moveMagnet(true);

   pinMode(ci_Mode_Button, INPUT);
   digitalWrite(ci_Mode_Button, true);
   updateScreen("Please    choose a  mode");
}

  
void loop()
{
  //Forward_Position_GoTo += 4;
  //Right_Position_GoTo += 4;
  //Angle_Offset +=5;
  //HallEffect();
  delay(10);//Don't remove this delay. Sending updates too quickly will block data returning.
  Serial.println(ultrasonic_back);
  /*programArm();
  return;/* */
  /*moveArm(8);
   * 
*/
  if((millis() - bt_timer) > 3000)
  {
    bt_Time_Up = true;
  }

  // button-based mode selection
  if(!digitalRead(ci_Mode_Button))
  {
    if(bt_Do_Once == false)
    {
      const char* modeTitles[3] = {"Mode 1", "Mode 2", "Diagnostic"};
      bt_Do_Once = true;
      ui_Mode_Index = (ui_Mode_Index+1)%3;
      bt_timer = millis();
      bt_Time_Up = false;
      updateScreen(modeTitles[ui_Mode_Index]);
      startPosition();
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }


  switch(ui_Mode_Index)
  {
    case 0:    //Robot stopped
    {
      if(bt_Time_Up)
      {
        Mode1();
      }
      break;
    }
    case 1:    //Robot stopped
    {
      if(bt_Time_Up)
      {
        Mode2();
      }
      break;
    }
    case 2:    //Robot stopped
    {
      if(bt_Time_Up)
      {
        DiagnosticsMode();
      }
      break;
    }
    default:
      UpdateUltrasonics();
      break;
  }
  
  return;
  
  // Setup PWM example.
  /*
  Serial.print(", ");
  Serial.print(ultrasonic_left);
  Serial.print(", ");
  Serial.print(ultrasonic_back);
  Serial.print(", ");
  Serial.println(ultrasonic_angle);
  
  Serial.print(ultrasonic_front_avg);
  Serial.print(", ");
  Serial.print(ultrasonic_left_avg);
  Serial.print(", ");
  Serial.print(ultrasonic_back_avg);
  Serial.print(", ");
  Serial.println(ultrasonic_angle_avg);
  UpdateUltrasonics();
  return;
  //*/
  
  /* MAIN CODE GOES HERE! :) */
  return;
  finalDropOff();
  return;
  
  //Serial.println(readForceSensor());
 
  //moveArm(1);
  //Serial.println("start");
  //programArm();
  
  //programArmPot();
  
  
}

//////////////////////////////////////////////////////////////////////////////

void DiagnosticsMode() {
  int numberOfIssues = 0;
  String issue = "Check:";
  
  getPositiion();

  if(abs(Left_Motor_Position) < 25)
    issue += " Left_Motor";
  if(abs(Right_Motor_Position) < 25)
    issue += " Right_Motor";
  if(abs(Front_Motor_Position) < 25)
    issue += " Front_Motor";
  if(abs(Back_Motor_Position) < 25)
    issue += " Back_Motor";
  
  UpdateUltrasonics();

  if(cm[0] == US_MAX)
    issue += " Upper_Left_US";
  if(cm[1] == US_MAX)
    issue += " Back_US";
  if(cm[2] == US_MAX)
    issue += " Lower_Left_US";
  if(cm[3] == US_MAX)
    issue += " Front_US";


  int sensorValue = analogRead(ci_HallEffect_Data);
  if(sensorValue>510||sensorValue<495) {
    issue += " HallEffect";
  }

  sensorValue = analogRead(ci_LineSensor_Data);
  if(sensorValue < 600) {
     issue += " LineSensor";
  }

  sensorValue = analogRead(forcePin);
  if(sensorValue < 600) {
     issue += " LineSensor";
  }
  
  if(issue == "Check:")
    issue = "Good to go";

  
  updateScreen(issue);
}

void Mode1() {
  static boolean driving_forward = true;
  UpdateUltrasonics();
  static boolean wentHome = false;
  static int last_Forward_Position_GoTo = 0;
  static int last_Right_Position_GoTo = 0;
  static int tesseractsPickedUp = 0;

  if(tesseractsPickedUp >= 3) {
    stopMotors();
    return;
  }

  if(ultrasonic_left_avg>1200) {
    moveArm(0);
    goHome();
    last_Forward_Position_GoTo = 0;
    last_Right_Position_GoTo = 0;
    driving_forward = true;
    return;
  }
  
  if(!wentHome) {
    //goHome();
    wentHome = true;
    moveMagnet(true);
  }
  int speedz = 12;
  if(driving_forward) {
    if(ultrasonic_front < 500 && ultrasonic_back > 1000)
      speedz = 5;
    if(ultrasonic_front < 300 && ultrasonic_back > 1000) {
      moveMagnet(false);
      slowMoveToPosition(Forward_Position_GoTo-150, Right_Position_GoTo-400, 90);
      driving_forward = false;
      stopMotors();
      moveArm(2);
      
    }
  }
  else {
    if(ultrasonic_back < 500 && ultrasonic_front > 1000)
      speedz = 5;
    if(ultrasonic_back < 300 && ultrasonic_front > 1000) {
       moveMagnet(false);
      slowMoveToPosition(Forward_Position_GoTo+150, Right_Position_GoTo-400, 90);
      driving_forward = true;
      stopMotors();
      moveArm(1);
    }
  }
  
  if (!holdingTesseract()) {
    moveArm((driving_forward?1:2));
    Forward_Position_GoTo += speedz*(driving_forward?1:-1);
  }
  else {
    tesseractsPickedUp += 1;
    last_Forward_Position_GoTo = Forward_Position_GoTo;
    last_Right_Position_GoTo = Right_Position_GoTo;
    if(tesseractsPickedUp == 1) {
      last_Right_Position_GoTo -= 800;
    }
    stopMotors();
    moveArm(0);
    goHome();
    searchForSpotAndDropOff();
    slowMoveToPosition(last_Forward_Position_GoTo, last_Right_Position_GoTo, 500);
  }
  
  
  updateMotors();
}

void Mode2() {
  searchCubeSpotAndPickup();
  finalDropOff();
  
}

void updateMotors() {
 updateMotors(true);
}

void updateMotors(boolean US) {
  getPositiion();
  ComputePIDs();
  writeToMotors();
  
  if(US && abs(ultrasonic_angle) < 100)
    Angle_Offset -= ultrasonic_angle/10;
}

void updateMotorsKeepDistanceFromLeft(int distance) {
  //Right_Position_GoTo = (Front_Motor_Position+Back_Motor_Position)/2 + (ultrasonic_left_avg-distance);
  Right_Position_GoTo += constrain((ultrasonic_left_avg-distance)/5, -10, 10);
  //if(Right_Position_
  updateMotors();
  UpdateUltrasonics();
}

void updateMotorsKeepDistanceFromLeftFar(int distance) {
  //Right_Position_GoTo = (Front_Motor_Position+Back_Motor_Position)/2 + (ultrasonic_left_avg-distance);
  Right_Position_GoTo += constrain((ultrasonic_left_avg-distance)/5, -4, 4);
  Right_Position_GoTo = constrain(Right_Position_GoTo, (Front_Motor_Position+Back_Motor_Position)/2 - 100, (Front_Motor_Position+Back_Motor_Position)/2 + 100);
  //if(Right_Position_
  updateMotors();
  UpdateUltrasonics();
}

void motorEncoderInit() {

  encoder_BackMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_BackMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_FrontMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_FrontMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward 
  
}
//Motors
void writeToMotors() {
  leftMotor.write(1500 - Left_Motor_Speed);
  rightMotor.write(1500 - Right_Motor_Speed);
  frontMotor.write(1500 - Front_Motor_Speed);
  backMotor.write(1500 - Back_Motor_Speed);
}

void stopMotors() {
  leftMotor.write(1500);
  rightMotor.write(1500);
  frontMotor.write(1500);
  backMotor.write(1500);
}

void ComputePIDs() {
  Forward_Position_GoTo += Angle_Offset;
  leftPID.Compute();
  Forward_Position_GoTo -= 2*Angle_Offset;
  rightPID.Compute();
  Forward_Position_GoTo += Angle_Offset;
  
  Right_Position_GoTo += Angle_Offset*1.2;
  frontPID.Compute();
  Right_Position_GoTo -= 2*(Angle_Offset*1.2);
  backPID.Compute();
  Right_Position_GoTo += Angle_Offset*1.2;
}

void startPosition() {
  encoder_LeftMotor.zero();
  encoder_RightMotor.zero();  
  encoder_FrontMotor.zero();
  encoder_BackMotor.zero();
}

void updateEncoder(double &curPosition, double newPos) {
  const int maxChange = 100;
  if (abs(newPos-curPosition) < maxChange && newPos != -1 && newPos != -0  && newPos != 1)
    curPosition = newPos;
}

void getPositiion(){
  updateEncoder(Left_Motor_Position, encoder_LeftMotor.getRawPosition());
  updateEncoder(Right_Motor_Position, encoder_RightMotor.getRawPosition());
  updateEncoder(Front_Motor_Position, encoder_FrontMotor.getRawPosition());
  updateEncoder(Back_Motor_Position, encoder_BackMotor.getRawPosition());
  
  Serial.print(Left_Motor_Position);
  Serial.print(", ");
  Serial.print(Right_Motor_Position);
  Serial.print(", ");
  Serial.print(Front_Motor_Position);
  Serial.print(", ");
  Serial.println(Back_Motor_Position);
}

void goHome() {
  const int back = 80;
  const int left = 40;
  const int angle = 10;
  
  while(ultrasonic_back_avg > back || ultrasonic_back == 0 || ultrasonic_left_avg > left || ultrasonic_left == 0) {
  //ultrasonic_angle
    UpdateUltrasonics();
    if(ultrasonic_back_avg > back) {
      if(abs(ultrasonic_back_avg - back) < 100)
        Forward_Position_GoTo -= 2;
      else
        Forward_Position_GoTo -= 6;
    }
    if(ultrasonic_left_avg > left) {
      if(abs(ultrasonic_left_avg - left) < 100)
        Right_Position_GoTo += 2;
      else
        Right_Position_GoTo += 6;
    }
    updateMotors();
    delay(10);
  }
  /*startPosition();
  Forward_Position_GoTo = 0;
  Right_Position_GoTo = 0;*/
  
}

void searchForSpotAndDropOff() {
  const int moveforward = 1000;
  const int steps = 300;
  const int leftDistance = 40;
  
  slowMoveToPosition(Forward_Position_GoTo+moveforward, Right_Position_GoTo, 300, leftDistance);
  moveArm(3);
  for(int i = 0; i < steps; i++) {
    Forward_Position_GoTo -= moveforward/steps;
    if(HallEffect())
      break;
    updateMotorsKeepDistanceFromLeft(leftDistance);
  }
  
  const int moveforwardblackline = 400;
  for(int i = 0; i < 200; i++) {
    Forward_Position_GoTo += moveforwardblackline/200;
    if(DetectBlackLine())
      break;
    updateMotorsKeepDistanceFromLeft(leftDistance);
  }
  slowMoveToPosition(Forward_Position_GoTo+100, Right_Position_GoTo-220, 100);
  stopMotors();
  moveArm(4);
  delay(3000);
  moveMagnet(false);
  
  delay(1000);
  moveArm(0);
  delay(2000);
  //goHome();
}

boolean searchCubeSpotAndPickup() {
  moveMagnet(true);
  const int moveforward = 1700;
  const int steps = 500;
  const int leftDistance = 90;

  for(int i = 0; i < 150; i++) {
    Forward_Position_GoTo -= moveforward/200;
    updateMotors(false);
  }

  int originalPos = Forward_Position_GoTo;

  boolean detect = false;
  while(!detect) {
    
  for(int i = 0; i < 200; i++) {
    Forward_Position_GoTo -= moveforward/200;
    updateMotors(false);
  }
  moveArm(19);
  
  for(int i = 0; i < steps; i++) {
    Forward_Position_GoTo += moveforward/steps;
    if(holdingTesseract()) {
      detect = true;
      break;
    }
    updateMotorsKeepDistanceFromLeft(leftDistance);
  }
  if(!detect) {
    stopMotors();
    moveArm(0);
    delay(5000);
  }
  }

  stopMotors();
  elbowServo.write(10, 30,true);
  shoulderServo.write(120, 10, true);
  moveArm(5);
  stopMotors();


  while(Forward_Position_GoTo < originalPos) {
    Forward_Position_GoTo += moveforward/steps;
    updateMotorsKeepDistanceFromLeftFar(100);
  }
  for(int i = 0; i < 100;i++) {
    updateMotorsKeepDistanceFromLeftFar(100);
    delay(10);
  }
  //slowMoveToPosition(0, 0, 300, false);
  return true;
}



void slowMoveToPosition(int forw, int right, int steps) {
  slowMoveToPosition(forw, right, steps, 0);
}

void slowMoveToPosition(int forw, int right, int steps, boolean US) {
  slowMoveToPosition(forw, right, steps, 0, US);
}

void slowMoveToPosition(int forw, int right, int steps, int keepLeft) {
  slowMoveToPosition(forw, right, steps, keepLeft, true);
}

void slowMoveToPosition(int forw, int right, int steps, int keepLeft, boolean US) {
  int o_Forward_Position_GoTo = Forward_Position_GoTo;
  int o_Right_Position_GoTo = Right_Position_GoTo;
  forw = forw-Forward_Position_GoTo;
  right = right-Right_Position_GoTo;
  for(int i = 1; i <= steps; i++) {
    UpdateUltrasonics();
    Forward_Position_GoTo = quinticEase(i, o_Forward_Position_GoTo, forw, steps);
    Right_Position_GoTo = quinticEase(i, o_Right_Position_GoTo, right, steps);
    if(keepLeft>0)
      updateMotorsKeepDistanceFromLeft(keepLeft);
    else
      updateMotors(US);
    delay(10);
  }
}

void finalDropOff() {
  Serial.println("finalDropOff");
  stopMotors();
  const int moveforward = 20000;
  const int steps = 3000;
  for(int i = 0; i < steps; i++) {
    Forward_Position_GoTo += moveforward/steps;
    if(DetectDropoffBar())
      break;
    updateMotors(false);
  }
  slowMoveToPosition(Forward_Position_GoTo+500, Right_Position_GoTo, 200, false);
  stopMotors();
  /*leftMotor.detach();
  rightMotor.detach();
  frontMotor.detach();
  backMotor.detach();*/
  moveArm(6);
  delay(2000);
  /*leftMotor.attach(leftMotorPin, 500, 2500);
  rightMotor.attach(rightMotorPin, 500, 2500);
  frontMotor.attach(frontMotorPin, 500, 2500);
  backMotor.attach(backMotorPin, 500, 2500);*/
  slowMoveToPosition(Forward_Position_GoTo-650, Right_Position_GoTo, 200, false);
  stopMotors();
  moveArm(7);
  delay(2000);
  moveMagnet(false);
  delay(2000);
  moveArm(6);
  delay(2000);
  moveArm(10);
  moveArm(11);
  delay(2000);
  slowMoveToPosition(Forward_Position_GoTo-650, Right_Position_GoTo, 200, false);
  stopMotors();
}

void programArm() {
  int arm = 0;
  VarSpeedServo* servo;
  int degree = 0;
  while (Serial.available() == 0);
  arm = Serial.parseInt();
  if(arm == 1)
    servo = &torsoServo;
  else if (arm == 2)
    servo = &shoulderServo;
  else
     servo = &elbowServo;
   Serial.flush();
   while (Serial.available() == 0);
   degree = Serial.parseInt();
   servo->write(degree);
   
   Serial.print("moved ");
   Serial.print(arm);
   Serial.print(" to ");
   Serial.println(degree);
   Serial.flush();
}

void programArmPot() {
  moveMagnet (true);
  int angle1 = analogRead(A1)/1023.0*180.0;
  int angle2 = analogRead(A2)/1023.0*180.0;
  int angle3 = analogRead(A3)/1023.0*180.0;
  torsoServo.write(angle1);
  shoulderServo.write(angle2);
  elbowServo.write(angle3);
  /*
  Serial.print("(");
   Serial.print(angle1);
   Serial.print(", ");
   Serial.print(angle2);
   Serial.print(", ");
   Serial.print(angle3);
   Serial.println(")");*/
}


const int US_NUMB = 5;
// measure distance to target using ultrasonic sensor  
void UpdateUltrasonics()
{
  Wire.requestFrom(8, 8);    // request 6 bytes from slave device #8

  //static int cm_[4][US_NUMB] = {{999999,999999,999999,999999,999999},{999999,999999,999999,999999,999999},{999999,999999,999999,999999,999999},{999999,999999,999999,999999,999999}};
  static int cm_[4][US_NUMB] = {{US_MAX,US_MAX,US_MAX},{US_MAX,US_MAX,US_MAX},{US_MAX,US_MAX,US_MAX},{US_MAX,US_MAX,US_MAX}};
  static int cm_i[4] = {0};
  
  for(int i = 0; i < 4; i++) 
    cm[i]=(Wire.read()<<8)+Wire.read();
  for(int i = 0; i < 4; i++) {
    if(cm[i] == 0 || cm[i]==cm_[i][cm_i[i]])
      continue;
     cm_i[i] = (cm_i[i]+1)%US_NUMB;
     cm_[i][cm_i[i]] = cm[i];
      
  }
  
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < US_NUMB; j++) {
      if(cm_[i][j] > cm[i])
        cm[i] = cm_[i][j];
    } 
  }
  
  int cm_avg[4] = {0};
  
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < US_NUMB; j++) {
        cm_avg[i] += cm_[i][j];
    } 
  }
  
  ultrasonic_front_avg = (cm_avg[3])/1/US_NUMB;
  ultrasonic_left_avg = (cm_avg[0]+cm_avg[2])/2/US_NUMB;
  ultrasonic_back_avg = cm_avg[1]/1/US_NUMB;
  ultrasonic_angle_avg = (cm_avg[0]-cm_avg[2])/1/US_NUMB;
  
  ultrasonic_front = (cm[3])/1;
  ultrasonic_left = (cm[0]+cm[2])/2;
  ultrasonic_back = cm[1]/1;
  ultrasonic_angle = (cm[0]-cm[2])/1;
  
}

// HallEffect 
boolean HallEffect()
{
  int sensorValue = analogRead(ci_HallEffect_Data);

  Serial.print("Hall Effect Value: ");
  Serial.println(sensorValue, DEC);
  if(sensorValue>520||sensorValue<490) {
    return true;
  }
  return false;
}

// HallEffect 
boolean HallEffectSensitive()
{
  int sensorValue = analogRead(ci_HallEffect_Data);

  Serial.print("Hall Effect Value: ");
  Serial.println(sensorValue, DEC);
  if(sensorValue>510||sensorValue<495) {
    return true;
  }
  return false;
}

// detect black line  
boolean DetectBlackLine()
{
  int sensorValue = analogRead(ci_LineSensor_Data);

Serial.print("Blackline Value: ");
  Serial.println(sensorValue, DEC);
  if(sensorValue > 500) {
     return true;
  }
  return false;
}

boolean DetectDropoffBar()
{
  int sensorValue = analogRead(ci_LineSensor_Data);
  if(sensorValue < 500) {
     return true;
  }
  return false;
}

void moveArm(int pos)           //moves arm to different positions
{
  if (pos == 0) {
    //torsoServo.write(127, 30);
    moveMagnet(true);
    shoulderServo.write(55, 30, true);
    elbowServo.write(180, 30);
  }
  else if (pos == 1)   //searching
  {
    servoSequencePoint seq[] = {{160,27},{110,27}};
    torsoServo.sequencePlay(seq, 2, true, 0);
    moveMagnet(true);
    shoulderServo.write(175, 30, true);
    elbowServo.write(175, 30, true);
    moveMagnet(true);
  }
  else if (pos == 2)   //searching backwards
  {
    servoSequencePoint seq[] = {{110,27},{160,27}};
    torsoServo.sequencePlay(seq, 2, true, 0);
    moveMagnet(true);
    shoulderServo.write(43, 30, true);
    elbowServo.write(16, 30, true);
    moveMagnet(true);
  }
  else if (pos == 3)    //dropping off(scanning)
  {
    torsoServo.write(15, 30);
    shoulderServo.write(180, 30);
    elbowServo.write(10, 30);
  }
  else if (pos == 4)  //dropping off
  {
    torsoServo.write(35, 30);
    shoulderServo.write(149, 30);
    elbowServo.write(180, 30);
  }
  else if (pos == 5)  //going under bridge
  {
    torsoServo.write(125, 30);
    shoulderServo.write(41, 30);
    elbowServo.write(50, 30);
  }
  else if (pos == 6)  //going under bridge
  {
    torsoServo.write(125, 30);
    shoulderServo.write(90, 30);
    elbowServo.write(20, 30);
  }
  else if (pos == 7)  //going under bridge
  {
    torsoServo.write(125, 30);
    shoulderServo.write(75, 30);
    elbowServo.write(140, 30);
  }
  else if (pos == 8)  //extended
  {
    torsoServo.write(60, 30);
    shoulderServo.write(180, 30);
    elbowServo.write(10, 30);
  }
  else if (pos == 9)  //extended
  {
    servoSequencePoint seq[] = {{55,10},{65,10}};
    torsoServo.sequencePlay(seq, 2, true, 0);
    shoulderServo.write(180, 30);
    elbowServo.write(103, 30);
  }
  else if (pos == -1)  //going under bridge
  {
    torsoServo.write(125, 30);
    shoulderServo.write(41, 30);
    elbowServo.write(90, 30);
  }
  else if (pos == 10)  //going under bridge
  {
    torsoServo.write(125, 30);
    elbowServo.write(0, 30, true);
    shoulderServo.write(120, 30, true);
  }
  else if (pos == 11)  //going under bridge
  {
    torsoServo.write(125, 30);
    shoulderServo.write(180, 30);
    elbowServo.write(90, 30);
  }
  else if (pos == 19)  //extended
  {
    torsoServo.write(105, 30);
    shoulderServo.write(180, 30);
    elbowServo.write(103, 30);
  }
}

// t: current time, b: beginning value, c: change in value, d: duration
float quinticEase(float t, float b, float c, float d) {
  /*t /= d;
  t--;
  return c*(t*t*t*t*t + 1) + b;*/
 t /= d;
 return -c * t*(t-2) + b;
};

boolean holdingTesseract() {
  return readForceSensor() < 500;
}

int readForceSensor()           //returns value from the force sensor
{
  int forceReading = analogRead(forcePin);
  Serial.print("Analog reading = ");
  Serial.println(forceReading);
  return forceReading;
}

void moveMagnet (boolean extended)      //extends and retracts magnet
{
  if (extended)
    handServo.write(145);
  else
    handServo.write(25);
}

void updateScreen(String str) {
  Wire.beginTransmission(4);
  Wire.write(str.c_str());
  Wire.endTransmission();
}
