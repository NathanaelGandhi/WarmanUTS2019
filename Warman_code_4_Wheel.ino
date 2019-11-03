//Pin definitions
#define m1A 22
#define m1B 23
#define m1 4  // motor 1 pwm pin
#define m2A 24
#define m2B 25
#define m2 5  // motor 2 pwm pin
#define m3A 26
#define m3B 27
#define m3 6 // motor 3 pwm pin
#define m4A 29
#define m4B 28
#define m4 7 // motor 4 pwm pin
#define m5A 30
#define m5B 31
#define m5 8 // motor 5 pwm pin
#define switch1 21  //black
#define switch2 48  //Yellow
#define switch3 50  //Blue
#define switch4 52  //Green
#define switch5 32  //Front Collector
#define encoderA1 2
#define encoderA2 3
#define encoderA3 18
#define encoderA4 19
#define encoderA5 20
#define ir1 44
#define ir2 35

#define RADIUS 114.5
#define Pi 3.14159265359

#define DISDOWN1 80   // in mm
#define DISDOWN2 DISUP-DISDOWN1  // down polls switch 3 for the bottom also
#define DISUP 140   // old val 1650


#include <Servo.h>
Servo myservo2;

const bool UP = true;
const boolean CCW = true;

volatile int g_mode = 0;
volatile int g_pressed = 0;
volatile int g_cwSpeed = 0;
volatile int g_ccwSpeed = 0;
volatile int g_speed_offset1 = 0;
volatile int g_speed_offset2 = 0;
volatile int g_speed_offset3 = 0;
volatile int g_speed_offset4 = 0;
volatile int g_speed_offset5 = 0;
volatile long g_encoder_count1 = 0;
volatile long g_encoder_count2 = 0;
volatile long g_encoder_count3 = 0;
volatile long g_encoder_count4 = 0;
volatile long g_encoder_count5 = 0;
volatile long g_start_time = 0;
volatile boolean g_switch1_pressed = false;
volatile boolean g_driving = false;
volatile boolean g_stop = false;

volatile int g_rotation_offset = -5;

int g_max_speed = 255;
int dis = 0;
int angle = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Omni-Bot Control Begin");
  pinMode(m1A, OUTPUT);
  pinMode(m1B, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2A, OUTPUT);
  pinMode(m2B, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3A, OUTPUT);
  pinMode(m3B, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4A, OUTPUT);
  pinMode(m4B, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(m5A, OUTPUT);
  pinMode(m5B, OUTPUT);
  pinMode(m5, OUTPUT);
  pinMode(encoderA1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderA3, INPUT);
  pinMode(encoderA4, INPUT);
  pinMode(encoderA5, INPUT);
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(switch4, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA1), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA2), encoder2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA3), encoder3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA4), encoder4ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA5), encoder5ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(switch1), switch1PressedISR, CHANGE);
  myservo2.attach(9);  // attaches the servo on pin 8 to the servo object
}

//===================================================
void loop()
{
  //(fwd(), bkw(), rt(), lt(), cw(), ccw(), stp())
  switch (g_mode) {
    case 0:   // INIT
      resetSwitch1();
      //      ServoRotateHome();
      //                              LeadScrew(UP, 50, g_max_speed); stp();   ///TESTING CODE/
      Serial.println();
      Serial.println("Press SWITCH1 to home.");
     // Serial.println("Press SWITCH3 to start.");
      int collectorSwitch;
      while (digitalRead(switch3)) {
        // Hang out till we are ready to go - To start press switch3
        collectorSwitch = digitalRead(switch4);
        Serial.println(collectorSwitch);
        delay(250);
        if (g_switch1_pressed)    // Press SWITCH1 to run HOMING
        {
          delay(1000);
          LeadScrewHome();
          ServoRotateHome();
          resetSwitch1();
          Serial.println("Press SWITCH1 to home.");
          Serial.println("Press SWITCH3 to start.");
        }
      }
      delay(3000);
      g_start_time = millis();
      g_mode = 1;
      break;
    case 1:   //DRIVE FORWARDS
      irfwd(g_max_speed);   //Run into the wall
      //      irAlignToCompound();
      irfwd(g_max_speed);   //just make sure we are still against it
      g_mode = 2;
      break;
    case 2:   // LOWER COLLECTOR
      Serial.println("Moving Down. STAGE 1");
      LeadScrew(!UP, DISDOWN1, g_max_speed);  // move collector down to just above the balls
      stp();
      localiseInCompound(); //move left
      stp();
      //      irAlignToCompound();
      Serial.println("Moving Down. STAGE 2");
      LeadScrew(!UP, DISDOWN2, g_max_speed ); // move collector all the way down
      stp();
      delay(500);
      g_mode = 3;
      break;
    case 3:   // DRIVE SIDEWAYS
      rt(850, g_max_speed); // Drive Sideways
      delay(200);
      stp();
      // align to compound. too risky to rotate, just drive forwards. forwards has switch1 & switch2 check
      //      g_switch1_pressed = false;
      irfwd(g_max_speed);
      //      irAlignToCompound();
      //      fwd(200, g_max_speed);
      //      stp();
      g_mode = 8;
      break;
    case 4:   //PICK UP COLLECTOR
      Serial.println("Moving Up 50");
      LeadScrew(UP, 30, g_max_speed); //move collector up
      //      stp();
      lt(30, g_max_speed);
      stp();
      Serial.println("Tilting Up to 70");
      ServoRotate(75 + g_rotation_offset); // tilt collector up
      delay(100);
      rt(30, g_max_speed);
      stp();
      Serial.println("Moving Up MAX");
      LeadScrew(UP, DISUP - 30, g_max_speed);
      stp();
      ServoRotate(65 + g_rotation_offset);
      stp();
      delay(500);
      g_mode = 5;
      break;
    case 5:   //TURN AROUND
      irfwd(g_max_speed);
      //      irAlignToCompound();
      bkw(1000, g_max_speed);   // Go bac1kwards
      stp();
      Serial.println("Tilting Up");
      ServoRotate(50 + g_rotation_offset); // tilt collector up
      stp();
      cw(200, g_max_speed / 4); // turn around
      stp();
      g_mode = 6;
      break;
    case 6:   //DRIVE TO OTHER COMPOUND & DELIVER PAYLOADS
      irfwd(g_max_speed);   //Run into the wall
      //      irAlignToCompound();
      irfwd(g_max_speed);   //make sure to be up against it
      Serial.println("Tilting Down");
      ServoRotate(170 + g_rotation_offset); // Empty the collector
      g_mode = 7;
      break;
    case 7:  //END SEQUENCE
      delay(5000);      // TESTING CODE!!!!
      Serial.println("Resetting tilt for another run");
      ServoRotate(90 + g_rotation_offset); // TESTING CODE!!!!!
      stp(); // End
      g_mode = 0;
      break;
    case 8: //COLLECT MISSED BALLS
      for (int i = 0; i < 2; i++)
      {
        lt(300, g_max_speed);
        stp();
        delay(1000);
        rt(400, g_max_speed);
        if (i == 1) {
          rt(100, g_max_speed);
        }
        stp();

        delay(1000);
      }
      g_mode = 4;
      break;
    case 9: // Collect missed balls, Scoop Up Method
      int distance = 150;
      int zoffset = 50;
      int method = 0;
      if (method == 0)
        for (int i = 0; i < 3; i++)
        {
          lt(distance, g_max_speed);
          stp();
          LeadScrew(UP, zoffset, g_max_speed);
          stp();
          delay(500);
          ServoRotate(60 + g_rotation_offset); // tilt collector up
          stp();
          delay(500);
          ServoRotate(90 + g_rotation_offset);
          delay(200);
          LeadScrew(!UP, zoffset, g_max_speed);
          stp();
          rt(distance, g_max_speed);
          stp();
          delay(500);
        }
      g_mode = 4;
      break;
  }
}
//===================================================

void encoder1ISR()
{
  g_encoder_count1++;
}
void encoder2ISR()
{
  g_encoder_count2++;
}
void encoder3ISR()
{
  g_encoder_count3++;
}
void encoder4ISR()
{
  g_encoder_count4++;
}
void encoder5ISR()
{
  g_encoder_count5++;
}

void switch1PressedISR()
{
  g_switch1_pressed = true;
  Serial.println("Switch1 Pressed");
}


void localiseInCompound()
{
  int distance = 20;
  int spd = g_max_speed / 1.3;
  Serial.println("Bot moving left");
  g_driving = true;
  while (g_driving && digitalRead(switch4)) {
    balanceMotors(); // Gives offset for motors
    astopMotor();
    bcwMotor(spd);
    cstopMotor();
    dcwMotor(spd);
  }
  Serial.println("Limit Reached()");
}
void irAlignToCompound()
{
  Serial.println("irAlignToCompound()");
  bstopMotor();
  dstopMotor();
  for (int i = 0; i < 6; i++)
  {
    Serial.println("Motor A GO");
    accwMotor(g_max_speed);
    delay(250);
    Serial.println("Motor A STOP");
    astopMotor();
    Serial.println("Motor C GO");
    ccwMotor(g_max_speed);
    delay(250);
    Serial.println("Motor C STOP");
    cstopMotor();
  }
}

//void alignToCompound()
//{
//  int readSwitch2 = digitalRead(switch2);
//  while (!g_switch1_pressed || readSwitch2)
//  {
//    fwd(30, g_max_speed);
//    readSwitch2 = digitalRead(switch2);
//    //    fwd(30, g_max_speed);
//    //    stp();
//    //    delay(100);
//    //    readSwitch2 = digitalRead(switch2);
//    //    bkw(20, g_max_speed);   // Go backwards
//    //    stp();
//    //    delay(100);
//
//    if (g_switch1_pressed && digitalRead(switch2))
//    {
//      cw(5, g_max_speed / 10);
//    }
//    if (!g_switch1_pressed && !digitalRead(switch2))
//    {
//      ccw(5, g_max_speed / 10);
//    }
//  }
//}
void resetSwitch1()
{
  if (g_switch1_pressed == true) {
    g_switch1_pressed = false;
  }
}

void balanceMotors()
{
  // find motor_number with highest % complete
  int motor_number = 0;
  int offset = 6;
  int scale = 5;
  // find motor_number with lowest % complete
  if (g_encoder_count1 < g_encoder_count2 && g_encoder_count1 < g_encoder_count3 && g_encoder_count1 < g_encoder_count4)
  {
    motor_number = 1;
  }
  if (g_encoder_count2 < g_encoder_count1 && g_encoder_count2 < g_encoder_count3 && g_encoder_count2 < g_encoder_count4)
  {
    motor_number = 2;
  }
  if (g_encoder_count3 < g_encoder_count1 && g_encoder_count3 < g_encoder_count2 && g_encoder_count3 < g_encoder_count4)
  {
    motor_number = 3;
  }
  if (g_encoder_count4 < g_encoder_count1 && g_encoder_count4 < g_encoder_count2 && g_encoder_count4 < g_encoder_count3)
  {
    motor_number = 4;
  }
  switch (motor_number) // speed up
  {
    case 1:
      g_speed_offset1 = g_speed_offset1 + offset;
      g_speed_offset2 = g_speed_offset2 - ((g_speed_offset2 / g_speed_offset1) * offset * scale);
      g_speed_offset3 = g_speed_offset3 - ((g_speed_offset3 / g_speed_offset1) * offset * scale);
      g_speed_offset4 = g_speed_offset4 - ((g_speed_offset4 / g_speed_offset1) * offset * scale);

      break;
    case 2:
      g_speed_offset2 = g_speed_offset2 + offset;
      g_speed_offset1 = g_speed_offset1 - ((g_speed_offset1 / g_speed_offset2) * offset * scale);
      g_speed_offset3 = g_speed_offset3 - ((g_speed_offset3 / g_speed_offset2) * offset * scale);
      g_speed_offset4 = g_speed_offset4 - ((g_speed_offset4 / g_speed_offset2) * offset * scale);
      break;
    case 3:
      g_speed_offset3 = g_speed_offset3 + offset;
      g_speed_offset2 = g_speed_offset2 - ((g_speed_offset2 / g_speed_offset3) * offset * scale);
      g_speed_offset1 = g_speed_offset1 - ((g_speed_offset1 / g_speed_offset3) * offset * scale);
      g_speed_offset4 = g_speed_offset4 - ((g_speed_offset4 / g_speed_offset3) * offset * scale);
      break;
    case 4:
      g_speed_offset4 = g_speed_offset4 + offset;
      g_speed_offset2 = g_speed_offset2 - ((g_speed_offset2 / g_speed_offset4) * offset * scale);
      g_speed_offset3 = g_speed_offset3 - ((g_speed_offset3 / g_speed_offset4) * offset * scale);
      g_speed_offset1 = g_speed_offset1 - ((g_speed_offset1 / g_speed_offset4) * offset * scale);
      break;
  }
}

int getPos()
{
  int pos1 = max(g_encoder_count1 * 0.442089, g_encoder_count2 * 0.442089);
  int pos2 = max(g_encoder_count4 * 0.442089, g_encoder_count3 * 0.442089);
  int pos = max(pos1, pos2);
  return pos;
}

int getLeadScrewPos()
{
  // 823.1 counts per revolution
  // 1028.875 counts per mm
  if (g_encoder_count5 == 0)
  {
    return 0;
  }
  else {
    int pos1 = g_encoder_count5 / 102.8875; //how many mm travel per encoder tick
    return pos1;
    // four starts is 8mm
    // two starts is 4mm
  }
}

void setDirection(int motor_number, boolean CCW)
{
  int aPin = 0;
  int bPin = 0;
  switch (motor_number)
  {
    case 1:
      aPin = m1A;
      bPin = m1B;
      break;
    case 2:
      aPin = m2A;
      bPin = m2B;
      break;
    case 3:
      aPin = m3A;
      bPin = m3B;
      break;
    case 4:
      aPin = m4A;
      bPin = m4B;
      break;
    case 5:
      aPin = m5A;
      bPin = m5B;
  }

  if (CCW)
  {
    digitalWrite(aPin, HIGH);
    digitalWrite(bPin, LOW);
  }
  else
  {
    digitalWrite(aPin, LOW);
    digitalWrite(bPin, HIGH);
  }
}

void setSpeed(int motor_number, int pwm_val)
{
  if (pwm_val > 255) {
    pwm_val = 255;
  }
  switch (motor_number)
  {
    case 1:
      analogWrite(m1, pwm_val);
      break;
    case 2:
      analogWrite(m2, pwm_val);
      break;
    case 3:
      analogWrite(m3, pwm_val);
      break;
    case 4:
      analogWrite(m4, pwm_val);
      break;
    case 5:
      analogWrite(m5, pwm_val);
      break;
  }
}

//HIGH LEVEL MOTOR FUNCTIONS (fwd, bkw, rt, lt, cw, ccw)

void irfwd(int spd) {
  Serial.println("Bot moving forward");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  int readir1 = 1;
  int readir2 = 1;
  bstopMotor();
  dstopMotor();
  accwMotor(spd);
  ccwMotor(spd);
  delay(1750);
  while (readir1 || readir2) {
    if (readir1)
    {
      accwMotor(spd);
      Serial.print("Motor A GO | ");
    }
    else
    {
      astopMotor();
      Serial.print("Motor A STOP | ");
    }
    if (readir2)
    {
      ccwMotor(spd);
      Serial.println("Motor C GO");
    }
    else
    {
      cstopMotor();
      Serial.println("Motor C STOP");
    }
    delay(250);
    readir1 = digitalRead(ir1);
    readir2 = digitalRead(ir2);
  }
  stp();
  delay(200);
}

//void fwd(int distance, int spd) {
//  Serial.println("Bot moving forward");
//  g_speed_offset1 = 0;
//  g_speed_offset2 = 0;
//  g_speed_offset3 = 0;
//  g_speed_offset4 = 0;
//  g_encoder_count1 = 0;
//  g_encoder_count2 = 0;
//  g_encoder_count3 = 0;
//  g_encoder_count4 = 0;
//  g_driving = true;
//  int pos = getPos();
//  while (pos < distance && g_driving) {
//    balanceMotors(); // Gives offset for motors
//    pos = getPos();
//    accwMotor(spd);
//    bstopMotor();
//    ccwMotor(spd);
//    dstopMotor();
//    if (g_switch1_pressed || millis() - g_start_time > 60000)
//    {
//      g_driving = false;
//    }
//    //    Serial.println(g_switch1_pressed);    Serial.print(g_switch2_pressed);
//
//  }
//  g_driving = false;
//}

void bkw(int distance, int spd) {
  Serial.println("Bot moving backwards");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  g_driving = true;
  int pos = getPos();
  while (pos < distance && g_driving) {
    balanceMotors(); // Gives offset for motors
    pos = getPos();
    acwMotor(spd);
    bstopMotor();
    cccwMotor(spd);
    dstopMotor();
  }
  g_driving = false;
}
void rt(int distance, int spd) {
  Serial.println("Bot moving right");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  g_driving = true;
  int pos = getPos();
  while (pos < distance && g_driving) {
    balanceMotors(); // Gives offset for motors
    pos = getPos();
    astopMotor();
    bccwMotor(spd);
    cstopMotor();
    dccwMotor(spd);
  }
}
void lt(int distance, int spd) {
  Serial.println("Bot moving left");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  g_driving = true;
  int pos = getPos();
  while (pos < distance && g_driving) {
    balanceMotors(); // Gives offset for motors
    pos = getPos();
    astopMotor();
    bcwMotor(spd);
    cstopMotor();
    dcwMotor(spd);
  }
}
void cw(int angle, int spd) {
  Serial.println("Bot moving clockwise");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  //  dis = (((RADIUS * Pi * 2) / 360) * angle) * 0.96;
  dis = (((RADIUS * Pi * 2) / 360) * angle) * 1;
  g_driving = true;
  int pos = getPos();
  while (pos < dis && g_driving) {
    balanceMotors(); // Gives offset for motors
    pos = getPos();
    accwMotor(spd);
    bccwMotor(spd);
    cccwMotor(spd);
    dcwMotor(spd);
  }
  g_driving = false;
}
void ccw(int angle, int spd) {
  Serial.println("Bot moving counter-clockwise");
  g_speed_offset1 = 0;
  g_speed_offset2 = 0;
  g_speed_offset3 = 0;
  g_speed_offset4 = 0;
  g_encoder_count1 = 0;
  g_encoder_count2 = 0;
  g_encoder_count3 = 0;
  g_encoder_count4 = 0;
  dis = (((RADIUS * Pi * 2) / 360) * angle) * 0.24;
  g_driving = true;
  int pos = getPos();
  while (pos < dis && g_driving) {
    balanceMotors(); // Gives offset for motors
    pos = getPos();
    accwMotor(spd);
    bcwMotor(spd);
    cccwMotor(spd);
    dccwMotor(spd);
  }
  g_driving = false;
}
void stp() {
  Serial.println("Bot stop");
  g_driving = false;
  astopMotor();
  bstopMotor();
  cstopMotor();
  dstopMotor();
  estopMotor();
  //  }
}

//LOW LEVEL MOTOR FUNCTIONS

void acwMotor(int spd) {
  //  Serial.println("Motor A clockwise");
  setDirection(1, !CCW);
  setSpeed(1, spd + g_speed_offset1);
}
void accwMotor(int spd) {
  //  Serial.println("Motor A counter-clockwise");
  setDirection(1, CCW);
  setSpeed(1, spd + g_speed_offset1);
}
void bcwMotor(int spd) {
  //  Serial.println("Motor B clockwise");
  setDirection(2, !CCW);
  setSpeed(2, spd + g_speed_offset2);
}
void bccwMotor(int spd) {
  //  Serial.println("Motor B counter-clockwise");
  setDirection(2, CCW);
  setSpeed(2, spd + g_speed_offset2);
}
void ccwMotor(int spd) {
  //  Serial.println("Motor C clockwise");
  setDirection(3, !CCW);
  setSpeed(3, spd + g_speed_offset3);
}
void cccwMotor(int spd) {
  //  Serial.println("Motor C counter-clockwise");
  setDirection(3, CCW);
  setSpeed(3, spd + g_speed_offset3);
}
void dcwMotor(int spd) {
  //  Serial.println("Motor D clockwise");
  setDirection(4, !CCW);
  setSpeed(4, spd + g_speed_offset4);
}
void dccwMotor(int spd) {
  //  Serial.println("Motor D counter-clockwise");
  setDirection(4, CCW);
  setSpeed(4, spd + g_speed_offset4);
}
void ecwMotor(int spd) {
  //  Serial.println("Motor E clockwise");
  setDirection(5, !CCW);
  setSpeed(5, spd + g_speed_offset5);
}
void eccwMotor(int spd) {
  //  Serial.println("Motor E counter-clockwise");
  setDirection(5, CCW);
  setSpeed(5, spd + g_speed_offset5);
}
void astopMotor() {
  //  Serial.println("Motor A stop");
  setSpeed(1, 0);
}
void bstopMotor() {
  //  Serial.println("Motor B stop");
  setSpeed(2, 0);
}
void cstopMotor() {
  //  Serial.println("Motor C stop");
  setSpeed(3, 0);
}
void dstopMotor() {
  //  Serial.println("Motor D stop");
  setSpeed(4, 0);
}
void estopMotor() {
  //  Serial.println("Motor E stop");
  setSpeed(5, 0);
}

void LeadScrew(bool dir, int distance, int spd) {
  g_encoder_count5 = 0;
  int pos = 0;
  if (dir == UP) {
    Serial.println("LeadScrew going UP");
    while (pos < distance)
    {
      eccwMotor(spd);
      delay(10);
      pos = getLeadScrewPos();
      Serial.println(pos);
    }
  }
  else {
    Serial.println("LeadScrew going DOWN");
    int readSwitch3 = digitalRead(switch3);
    while (pos < distance && readSwitch3)
    {
      readSwitch3 = digitalRead(switch3);
      ecwMotor(spd);
      delay(10);
      pos = getLeadScrewPos();
    }
  }
  delay(250);
}
void LeadScrewHome() {
  Serial.println("LeadScrewHome");
  LeadScrew(!UP, 1000, g_max_speed);
  stp();
  LeadScrew(UP, DISUP, g_max_speed);
  stp();
}

void ServoRotateHome() {
  Serial.println("ServoRotateHome");
  myservo2.write(40);
  delay(1000);
  angle = myservo2.read();
  delay(1000);
  ServoRotate(90 + g_rotation_offset);
  delay(1000);
}

void ServoRotate(int next_angle) {
  while (next_angle >= angle + 2 || next_angle <= angle - 2) {
    if (next_angle > angle) {
      angle = angle + 1;
    }
    else {
      angle = angle - 1;
    }
    myservo2.write(angle);
    delay(20);
  }
}
