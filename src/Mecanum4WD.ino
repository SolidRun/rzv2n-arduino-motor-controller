#include <Wire.h>
#include "PS2X_lib.h"
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
/////////////////////////////
//#include "Ultrasonic.h"
//////////////////////////// 
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
PS2X ps2x;
long ARM_MIN[]={10, 10, 40, 10};

long ARM_MAX[]={170, 140, 170, 102};

QGPMaker_Servo *Servo1 = AFMS.getServo(0);
QGPMaker_Servo *Servo2 = AFMS.getServo(1);
QGPMaker_Servo *Servo3 = AFMS.getServo(2);
QGPMaker_Servo *Servo4 = AFMS.getServo(3);
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);
QGPMaker_Encoder Enc1(1);
QGPMaker_Encoder Enc2(2);
QGPMaker_Encoder Enc3(3);
QGPMaker_Encoder Enc4(4);

enum RobotState {
  IDLE,
  MOVING,
  TURNING
};

enum MoveType {
  MOVE_FWD,
  MOVE_BWD,
  MOVE_LEFT,
  MOVE_RIGHT
};

RobotState state = IDLE;
long targetTicks = 0;

String input = "";
bool commandActive = false;
unsigned long commandEndTime = 0;

// Current action
String currentDirection = "";
int currentSpeed = 0;

float Kp_sync = 0;     
float Kp_slow = 0.05;   
int slowZone = 400;      
int minSpeed = 40;      

float motorGain[4] = {
  1.00,  // M1
  1.00,  // M2
  1.00,  // M3
  1.00   // M4
};

/////////////////////////////////////
// Ultrasonic ultrasonic = Ultrasonic(7,6);
// float getCmDistance(){
//   long t = ultrasonic.timing();
//   float cm = ultrasonic.convert(t, 1);
//   Serial.print("Distance: ");
//   Serial.print(cm);
//   Serial.println(" cm");
//   return cm;
// }
////////////////////////////////////
void resetEncoders() {
  Enc1.write(0); 
  Enc2.write(0); 
  Enc3.write(0); 
  Enc4.write(0);
}

void readEncoders(long t[4]) {
  t[0] = abs(Enc1.read());
  t[1] = abs(Enc2.read());
  t[2] = abs(Enc3.read());
  t[3] = abs(Enc4.read());
}

bool reachedTarget() {
   return abs(Enc1.read()) >= targetTicks &&
          abs(Enc2.read()) >= targetTicks &&
          abs(Enc3.read()) >= targetTicks &&
          abs(Enc4.read()) >= targetTicks;
}


void forwardSmart(long targetTicks, int maxSpeed) {

  Enc1.write(0); Enc2.write(0); Enc3.write(0); Enc4.write(0);

  while (true) {

    long t1 = abs(Enc1.read());
    long t2 = abs(Enc2.read());
    long t3 = abs(Enc3.read());
    long t4 = abs(Enc4.read());

    long avg = (t1 + t2 + t3 + t4) / 4;

    if (avg >= targetTicks) break;

  
    int baseSpeed = maxSpeed;
    long remaining = targetTicks - avg;

    if (remaining < slowZone) {
      baseSpeed = minSpeed + remaining * Kp_slow;
      baseSpeed = constrain(baseSpeed, minSpeed, maxSpeed);
    }

  
    //long minTicks = min(t1, min(t2, min(t3, t4)));
    int s1 = baseSpeed - Kp_sync * (t1 - avg);
    int s2 = baseSpeed - Kp_sync * (t2 - avg);
    int s3 = baseSpeed - Kp_sync * (t3 - avg);
    int s4 = baseSpeed - Kp_sync * (t4 - avg);

    s1 *= motorGain[0];   // M1
    s2 *= motorGain[1];   // M2
    s3 *= motorGain[2];   // M3
    s4 *= motorGain[3];   // M4

    s1 = constrain(s1, 0, 255);
    s2 = constrain(s2, 0, 255);
    s3 = constrain(s3, 0, 255);
    s4 = constrain(s4, 0, 255);

    DCMotor_1->setSpeed(s1); DCMotor_1->run(FORWARD);
    DCMotor_2->setSpeed(s2); DCMotor_2->run(FORWARD);
    DCMotor_3->setSpeed(s3); DCMotor_3->run(FORWARD);
    DCMotor_4->setSpeed(s4); DCMotor_4->run(FORWARD);

    delay(15);
  }


  DCMotor_1->setSpeed(0); DCMotor_1->run(BRAKE);
  DCMotor_2->setSpeed(0); DCMotor_2->run(BRAKE);
  DCMotor_3->setSpeed(0); DCMotor_3->run(BRAKE);
  DCMotor_4->setSpeed(0); DCMotor_4->run(BRAKE);

  delay(40);

  DCMotor_1->run(RELEASE);
  DCMotor_2->run(RELEASE);
  DCMotor_3->run(RELEASE);
  DCMotor_4->run(RELEASE);

  Serial.println("SMART STOP DONE");
}

void forward(uint8_t speed) {
    DCMotor_1->setSpeed(speed*motorGain[0]);
    DCMotor_1->run(FORWARD);
    DCMotor_4->setSpeed(speed*motorGain[3]);
    DCMotor_4->run(FORWARD);
    DCMotor_3->setSpeed(speed*motorGain[2]);
    DCMotor_3->run(FORWARD);
    DCMotor_2->setSpeed(speed*motorGain[1]);
    DCMotor_2->run(FORWARD);
}

void turnLeft(uint8_t speed) {
  DCMotor_1->setSpeed(speed);
  DCMotor_1->run(BACKWARD);
  DCMotor_4->setSpeed(speed);
  DCMotor_4->run(FORWARD);
  DCMotor_2->setSpeed(speed);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(speed);
  DCMotor_3->run(FORWARD);
}

void turnRight(uint8_t speed) {
  DCMotor_1->setSpeed(speed);
  DCMotor_1->run(FORWARD);
  DCMotor_4->setSpeed(speed);
  DCMotor_4->run(BACKWARD);
  DCMotor_2->setSpeed(speed);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(speed);
  DCMotor_3->run(BACKWARD);
}

void moveLeft(uint8_t speed) {
  DCMotor_1->setSpeed(speed*motorGain[0]);
  DCMotor_1->run(BACKWARD);
  DCMotor_4->setSpeed(speed*motorGain[3]);
  DCMotor_4->run(FORWARD);
  DCMotor_2->setSpeed(speed*motorGain[1]);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(speed*motorGain[2]);
  DCMotor_3->run(BACKWARD);
}

void moveRight(uint8_t speed) {
  DCMotor_1->setSpeed(speed*motorGain[0]);
  DCMotor_1->run(FORWARD);
  DCMotor_4->setSpeed(speed*motorGain[3]);
  DCMotor_4->run(BACKWARD);
  DCMotor_2->setSpeed(speed*motorGain[1]);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(speed*motorGain[2]);
  DCMotor_3->run(FORWARD);

}

void moveForwardAndLeft(uint8_t speed){
  DCMotor_2->setSpeed(speed);
  DCMotor_2->run(FORWARD);
  DCMotor_4->setSpeed(speed);
  DCMotor_4->run(FORWARD);
}

void moveForwardAndRight(uint8_t speed){
  DCMotor_1->setSpeed(speed);
  DCMotor_1->run(FORWARD);
  DCMotor_3->setSpeed(speed);
  DCMotor_3->run(FORWARD);  
}

void moveBackwardAndLeft(uint8_t speed){
  DCMotor_2->setSpeed(speed);
  DCMotor_2->run(BACKWARD);
  DCMotor_4->setSpeed(speed);
  DCMotor_4->run(BACKWARD);
}

void moveBackwardAndRight(uint8_t speed){
  DCMotor_1->setSpeed(speed);
  DCMotor_1->run(BACKWARD);
  DCMotor_3->setSpeed(speed);
  DCMotor_3->run(BACKWARD);  
}

void backward(uint8_t speed) {
  DCMotor_1->setSpeed(speed*motorGain[0]);
  DCMotor_1->run(BACKWARD);
  DCMotor_4->setSpeed(speed*motorGain[3]);
  DCMotor_4->run(BACKWARD);
  DCMotor_2->setSpeed(speed*motorGain[1]);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(speed*motorGain[2]);
  DCMotor_3->run(BACKWARD);
}

void stopMoving() {
  DCMotor_1->setSpeed(0);
  DCMotor_1->run(RELEASE);  
  DCMotor_2->setSpeed(0);
  DCMotor_2->run(RELEASE);
  DCMotor_4->setSpeed(0);
  DCMotor_4->run(RELEASE);
  DCMotor_3->setSpeed(0);
  DCMotor_3->run(RELEASE);

}

void brakeMoving(){
  DCMotor_1->setSpeed(0); DCMotor_1->run(BRAKE);
  DCMotor_2->setSpeed(0); DCMotor_2->run(BRAKE);
  DCMotor_3->setSpeed(0); DCMotor_3->run(BRAKE);
  DCMotor_4->setSpeed(0); DCMotor_4->run(BRAKE);

  delay(40);

  DCMotor_1->run(RELEASE);
  DCMotor_2->run(RELEASE);
  DCMotor_3->run(RELEASE);
  DCMotor_4->run(RELEASE);
}

void runRawMove(MoveType move, int speed) {
  switch (move) {

    case MOVE_FWD:
        forward(speed);
      break;

    case MOVE_BWD:
        backward(speed);
      break;

    case MOVE_LEFT:
      moveLeft(speed);
      break;

    case MOVE_RIGHT:
      moveRight(speed);
      break;

  }
}

void calibrateGain(
  MoveType move,
  int speed,
  long targetTicks,
  int sessions
) {
  float gainSum[4] = {0,0,0,0};

  for (int run = 0; run < sessions; run++) {

    resetEncoders();
    runRawMove(move, speed);

    while (true) {
      long t[4];
      readEncoders(t);

      if (t[0] >= targetTicks &&
          t[1] >= targetTicks &&
          t[2] >= targetTicks &&
          t[3] >= targetTicks)
        break;
    }

    brakeMoving();
    delay(150);

    long t[4];
    readEncoders(t);

    float ref = (t[0] + t[1] + t[2] + t[3]) / 4.0;

    for (int i = 0; i < 4; i++) {
      if (t[i] > 0)
        gainSum[i] += ref / t[i];
      else
        gainSum[i] += 1.0;
    }
  }

  for (int i = 0; i < 4; i++)
    motorGain[i] = gainSum[i] / sessions;

  float maxG = motorGain[0];
  for (int i = 1; i < 4; i++)
    if (motorGain[i] > maxG) maxG = motorGain[i];

  for (int i = 0; i < 4; i++)
    motorGain[i] /= maxG;
}

// void processCommand(String cmd) {
// //<direction>,<speed>,<time>
// //direction -  
// //MF-moveforward; MB-moveBackward; TL-turnLeft; TR-turnRight; 
// //ML-moveLeft; MR-moveRight; FR-moveForwardAndRight; FL-moveForwardAndLeft;
// //BR-moveBackwardAndRight; BL-moveBackwardAndLeft; SM-stopMoving;)
// //speed(20-255)
// //time(milisec)
//   if (commandActive) {
//     stopMoving();
//     commandActive = false;
//   }

//   cmd.trim();
  
//   int c1 = cmd.indexOf(',');
//   int c2 = cmd.indexOf(',', c1 + 1);

//   if (c1 == -1 || c2 == -1) {
//     Serial.println("Invalid command!");
//     return;
//   }

//   String direction = cmd.substring(0, c1);
//   int speed = cmd.substring(c1 + 1, c2).toInt();
//   unsigned long duration = cmd.substring(c2 + 1).toInt();

//   Serial.print("New command: ");
//   Serial.println(cmd);

  
//   currentDirection = direction;
//   currentSpeed = speed;
//   commandEndTime = millis() + duration;
//   commandActive = true;

//   // Act now
//   resetEncoders();
//   executeDirection(direction, speed);
// }

// void executeDirection(String direction, int speed) {

//   if (direction == "MF")      forward(speed);
//   else if (direction == "MB") backward(speed);
//   else if (direction == "TL") turnLeft(speed);
//   else if (direction == "TR") turnRight(speed);
//   else if (direction == "ML") moveLeft(speed);
//   else if (direction == "MR") moveRight(speed);
//   else if (direction == "FR") moveForwardAndRight(speed);
//   else if (direction == "FL") moveForwardAndLeft(speed);
//   else if (direction == "BR") moveBackwardAndRight(speed);
//   else if (direction == "BL") moveBackwardAndLeft(speed);
//   else if (direction == "SM") {
//     stopMoving();
//     commandActive = false;
//   } 
//   else {
//     Serial.println("Unknown direction!");
//   }
// }

void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "STOP") {
    brakeMoving();
    state = IDLE;
    Serial.println("DONE");
    return;
  }
  if (cmd == "READ") {
      Serial.println(Enc1.read());
      Serial.println(Enc2.read());
      Serial.println(Enc3.read());
      Serial.println(Enc4.read());
      return;
  }
  
  if (cmd == "SYNC") {
      calibrateGain(MOVE_FWD, 120, 3000, 6);
      delay(1000);
      Serial.print("motorGain: ");
      Serial.print(motorGain[0], 4);
      Serial.print(", ");
      Serial.print(motorGain[1], 4);
      Serial.print(", ");
      Serial.print(motorGain[2], 4);
      Serial.print(", ");
      Serial.println(motorGain[3], 4);
      return;
  }
  
  if (cmd == "FWDS") {
    forwardSmart(4000 , 150);
    return;
  }

  int c1 = cmd.indexOf(',');
  int c2 = cmd.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) {
    Serial.println("ERROR");
    return;
  }

  String type = cmd.substring(0, c1);
  int speed = cmd.substring(c1 + 1, c2).toInt();
  targetTicks = abs(cmd.substring(c2 + 1).toInt());
  resetEncoders();

  if (type == "FWD") {
    forward(speed);
    state = MOVING;
  }
  else if (type == "BWD") {
    backward(speed);
    state = MOVING;
  }
  else if (type == "TURN") {
    bool left = (cmd.substring(c2 + 1).toInt() > 0);
    if (left){
      moveLeft(speed);   
    }else{
      moveRight(speed); 
    }  
    state = TURNING;
  }
  else {
    Serial.println("ERROR");
    return;
  }

  Serial.println("BUSY");
}

void resetPCA9685() {
    Wire.beginTransmission(0x40);
    Wire.write(0x00);
    Wire.write(0x80); // reset
    Wire.endTransmission();
    delay(10);
    AFMS.begin(50);
}

void setup(){
  Serial.begin(9600);
  while (!Serial) { }
  Serial.println("READY");
  delay(500);
  resetPCA9685();
  Serial.println("MotorShield initialized");
  int error = 0;
  do{
    error = ps2x.config_gamepad(13,11,10,12, true, true);
    if(error == 0){
      Serial.println("Controller ready");
      break;
    }else{
      Serial.println("Controller not found.");
      delay(100);
    }
  }while(1);
  Serial.println("Arduino ready!");
}

void loop(){
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        processCommand(input);
        input = "";
      }
    } else {
      input += c;
    }
  }

  if (state != IDLE ) {
    if(reachedTarget() ){
      brakeMoving();
      state = IDLE;
      Serial.println("DONE");
    }
  }
  else{
    ps2x.read_gamepad(false, 0);
    delay(30);
    if (ps2x.Button(PSB_PAD_UP)) {
      if (ps2x.Button(PSB_L2)) {
        moveForwardAndLeft(50);
      } else if (ps2x.Button(PSB_R2)) {
        moveForwardAndRight(50);
      } else {
        forward(50);
      }

    } else if (ps2x.Button(PSB_PAD_DOWN)) {
      if (ps2x.Button(PSB_L2)) {
        moveBackwardAndLeft(50);
      } else if (ps2x.Button(PSB_R2)) {
        moveBackwardAndRight(50);
      } else {
        backward(50);
      }
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
      turnLeft(50);
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
      turnRight(50);
    } else if (ps2x.Button(PSB_L1)) {
      moveLeft(50);
    } else if (ps2x.Button(PSB_R1)) {
      moveRight(50);
    } else {
      stopMoving();
    }

    if (ps2x.Button(PSB_CROSS)) {
      ps2x.read_gamepad(true, 200);
      delay(30);
      ps2x.read_gamepad(false, 0);
    }
  }

  delay(2);

}



