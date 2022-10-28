
   
#include "Freenove_WS2812B_RGBLED_Controller.h"
#include "IRremote.h"

#define STRIP_I2C_ADDRESS  0x20
#define STRIP_LEDS_COUNT   10

#define IR_UPDATE_TIMEOUT     110
#define IR_CAR_SPEED          180

#define PIN_DIRECTION_RIGHT 3
#define PIN_DIRECTION_LEFT  4
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_MOTOR_PWM_LEFT  6

#define PIN_BATTERY     A0
#define PIN_BUZZER      A0

#define PIN_SONIC_TRIG    7    //define Trig pin
#define PIN_SONIC_ECHO    8    //define Echo pin
#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60) // calculate timeout 
#define SOUND_VELOCITY  340  //soundVelocity: 340m/s
#define PIN_IRREMOTE_RECV 9


#define IR_REMOTE_KEYCODE_POWER    0xFFA25D
#define IR_REMOTE_KEYCODE_MENU    0xFF629D
#define IR_REMOTE_KEYCODE_MUTE    0xFFE21D
#define IR_REMOTE_KEYCODE_MODE    0xFF22DD
#define IR_REMOTE_KEYCODE_UP      0xFF02FD
#define IR_REMOTE_KEYCODE_BACK    0xFFC23D
#define IR_REMOTE_KEYCODE_LEFT    0xFFE01F
#define IR_REMOTE_KEYCODE_CENTER  0xFFA857
#define IR_REMOTE_KEYCODE_RIGHT   0xFF906F
#define IR_REMOTE_KEYCODE_0       0xFF6897
#define IR_REMOTE_KEYCODE_DOWN    0xFF9867
#define IR_REMOTE_KEYCODE_OK      0xFFB04F
#define IR_REMOTE_KEYCODE_1     0xFF30CF
#define IR_REMOTE_KEYCODE_2     0xFF18E7
#define IR_REMOTE_KEYCODE_3     0xFF7A85
#define IR_REMOTE_KEYCODE_4     0xFF10EF
#define IR_REMOTE_KEYCODE_5     0xFF38C7
#define IR_REMOTE_KEYCODE_6     0xFF5AA5
#define IR_REMOTE_KEYCODE_7     0xFF42BD
#define IR_REMOTE_KEYCODE_8     0xFF4AB5
#define IR_REMOTE_KEYCODE_9     0xFF52AD
#define IR_REMOTE_KEYCODE_TEST  0xFF22DD

u8 colorPos = 0;
u8 colorStep = 50;
u8 stripDisplayMode = 1;
u8 currentLedIndex = 0;
u16 stripDisplayDelay = 100;
u32 lastStripUpdateTime = 0;
  
IRrecv irrecv(PIN_IRREMOTE_RECV);
decode_results results;
u32 currentKeyCode, lastKeyCode;
bool isStopFromIR = false;
u32 lastIRUpdateTime = 0;

int flagtostop = 0;
int distance = MAX_DISTANCE;
bool isBuzzered = false;
Freenove_WS2812B_Controller strip(STRIP_I2C_ADDRESS, STRIP_LEDS_COUNT, TYPE_GRB);


void initSonic(){
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
}

void setBuzzer(bool flag) {
  isBuzzered = flag;
  pinMode(PIN_BUZZER, flag);
  digitalWrite(PIN_BUZZER, flag);
}
void startReciever(){
  irrecv.enableIRIn();
}

void initLED()
{
  while (!strip.begin());

  strip.setAllLedsColor(0xFF0000); //Set all LED color to red
  delay(1000);
  strip.setAllLedsColor(0x00FF00); //set all LED color to gree
  delay(1000);
}

void initMotor(){
 pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
}

float getSonar() {   // Don't change this function  
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}

void motorRun(int speedl, int speedr) { // Don't change this function 
  int dirL = 0, dirR = 0;
  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }
  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

void getDistance(){
  distance = getSonar();   //get ultrsonice value and save it into distance[0]

}

void Movement(int period , int leftSpeed ,int rightSpeed) //MADE A NEW FUNCTION TO CHECK BOTH SONAR AND MOVE STRAIGHT 
{
  for(int i=0;i<period;i++){
    getDistance(); 
    if(distance<25){
      i--;
      motorRun(0,0);
      pinMode(PIN_BUZZER, true);
      digitalWrite(PIN_BUZZER, true);
     strip.setAllLedsColor(0xFF0000);
    }
    else {
      pinMode(PIN_BUZZER, false);
      digitalWrite(PIN_BUZZER, false);
      strip.setAllLedsColor(0x00FF00);
      motorRun(leftSpeed, rightSpeed);
      delay(2*period);
    }
  }
 }

void Buzzer(){
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
}
 
void setup() {
  Serial.begin(9600);
  setBuzzer(false);
  initMotor();
  initSonic();
  irrecv.enableIRIn();
}

void loop() {

  if (irrecv.decode(&results)){
    Serial.print(results.value, HEX);
    irrecv.resume();
  }
  delay(100);
  if (irrecv.decode(&results)) {
    isStopFromIR = false;
    currentKeyCode = results.value;
    if (currentKeyCode != 0xFFFFFFFF) {
      lastKeyCode = currentKeyCode;
    }
    switch (lastKeyCode) {
      case IR_REMOTE_KEYCODE_UP:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_DOWN:
        motorRun(-IR_CAR_SPEED, -IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_LEFT:
        motorRun(-IR_CAR_SPEED, IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_RIGHT:
        motorRun(IR_CAR_SPEED, -IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_CENTER:
         setBuzzer(true);
        break;
case IR_REMOTE_KEYCODE_TEST: 

         Movement(21,120,120); // THIS IS FOR 50 cm from start

         motorRun(180,-180);  // THIS IS FOR THE RIGHT TURN 
         delay(430);

         Movement(15,120,120); // THIS IS FOR 40 cm 

         motorRun(-150,150); // THIS IS FOR LEFT TURN 
         delay(550);

         Movement(19,120,120); //THIS IS FOR 50 cm

         motorRun(180,-180); //THIS IS FOR RIGHT TURN
         delay(440);

         Movement(18,160,160); //THIS IS FOR 60 cm

         motorRun(180,-180); //THIS IS FOR RIGHT 
         delay(440);

         Movement(24,160,160); //THIS IS FOR 100 cm

         motorRun(210,-210); //THIS IS FOR LEFT TURN 
         delay(430);

         Movement(25,160,160); // THIS IS FOR 100 cm which stops where it starts 

  break;
  case IR_REMOTE_KEYCODE_1:
        stripDisplayMode = 1;
        break;
      case IR_REMOTE_KEYCODE_2:
        colorStep += 5;
        if (colorStep > 100)
        {
          colorStep = 100;
        }
        break;
      case IR_REMOTE_KEYCODE_3:
        colorStep -= 5;
        if (colorStep < 5)
        {
          colorStep = 5;
        }
        break;
      case IR_REMOTE_KEYCODE_4:
        stripDisplayMode = 0;
        break;
      case IR_REMOTE_KEYCODE_5:
        stripDisplayDelay -= 20;
        if (stripDisplayDelay < 20)
        {
          stripDisplayDelay = 20;
        }
        break;
      case IR_REMOTE_KEYCODE_6:
        stripDisplayDelay += 20;
        if (stripDisplayDelay > 300)
        {
          stripDisplayDelay = 300;
        }
        break;
    }
     irrecv.resume();
     lastIRUpdateTime=millis();
  }
  else
  {if (millis() - lastIRUpdateTime > IR_UPDATE_TIMEOUT)
    {
      if (!isStopFromIR) {
        isStopFromIR = true;
        motorRun(0, 0);
        setBuzzer(false);
      }
      lastIRUpdateTime = millis();
    }
  }
  switch (stripDisplayMode)
  {
    case 0:
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        for (int i = 0; i < STRIP_LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(colorPos + i * 25));
        }
        strip.show();
        colorPos += colorStep;
        lastStripUpdateTime = millis();
      }
      break;
    case 1:
      if (millis() - lastStripUpdateTime > stripDisplayDelay)
      {
        strip.setLedColor(currentLedIndex, strip.Wheel(colorPos));
        currentLedIndex++;
        if (currentLedIndex == STRIP_LEDS_COUNT)
        {
          currentLedIndex = 0;
          colorPos += colorStep; //nrfDataRead[POT1] / 20;
        }
        lastStripUpdateTime = millis();
      }
      break;
    default:
      break;
  }
  }
 
   
