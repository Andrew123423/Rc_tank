// Adapted from supplied sketch by Dejan / HowToMechatronics. See THIRD_PARTY.md.
/*
      3D Printed RC Tank - Arduino Code
      by Dejan, www.HowToMechatronics.com

     Libraries:
     IBusBM: https://github.com/bmellink/IBusBM
*/


#define M1_RPWM 9
#define M1_LPWM 10
#define M2_RPWM 4
#define M2_LPWM 5

#define frontLedsNumber   18
#define backLeftLedsNumber   3
#define backRightLedsNumber   3

LedBuffer<18> frontLeds;
LedBuffer<3> backLeftLeds;
LedBuffer<3> backRightLeds;

PWMServo myservo;  // create servo object to control a servo

IBusBM IBus;
IBusBM IBusSensor;

int ch0, ch1, ch4, ch5, ch6, ch7, ch8, ch9 = 0;
int ch6State, ch7State, ch8State, ch9State = 1000;

int motorSpeed, steeringValue, leftMotorSpeed, rightMotorSpeed = 0;

int pot, out1, out2 = 0;
int i = 0;
int k = 0;
int k2 = 0;
int j = 0;
int r = 1;
int t = 0;
int i1 = 0;
int i1State = 0;

unsigned long previousMillis = 0;
unsigned long interval = 50;
unsigned long interval2 = 300;

void setup() {
  
  Serial.begin(115200);
  IBus.begin(Serial1, IBUSBM_NOTIMER); // Servo iBUS
  IBusSensor.begin(Serial2, IBUSBM_NOTIMER); // Sensor iBUS

  IBusSensor.addSensor(IBUSS_INTV); // add voltage sensor

  myservo.attach(13);  // attaches the servo on pin 9 to the servo object

  myservo.write(90);
  delay(1000);

  FastLED.addLeds<WS2812, 24, GRB>(frontLeds, frontLedsNumber);
  FastLED.addLeds<WS2812, 23, GRB>(backLeftLeds, backLeftLedsNumber);
  FastLED.addLeds<WS2812, 22, GRB>(backRightLeds, backRightLedsNumber);

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);

  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);

  for (int i = 0; i < frontLedsNumber; i++) {
    frontLeds[i] = CRGB ( 0, 0, 0);
  }
  for (int i = 0; i < backLeftLedsNumber; i++) {
    backLeftLeds[i] = CRGB ( 0, 0, 0);
    backRightLeds[i] = CRGB ( 0, 0, 0);
  }
  FastLED.show();
}

void loop() {


  // Reading the data comming from the RC Transmitter
  IBus.loop();
  ch0 = IBus.readChannel(0); // ch0 - left and right;
  ch1 = IBus.readChannel(1); // ch1 - forward and backward;

  ch6 = IBus.readChannel(6); // ch6 - Gear shifter
  ch7 = IBus.readChannel(7); // ch7 - Police light strobe effect
  ch8 = IBus.readChannel(8); // ch8 - All white LEDs
  ch9 = IBus.readChannel(9); // ch9 - Left and Right white LEDs

  // convert the incoming date into suitable PWM value
  steeringValue = map(ch0, 1000, 2000, -205, 205); // 0 to 205 range because then I add +50 in order to avoid low PWM values as to motors won't start if so
  motorSpeed = map(ch1, 1000, 2000, -205, 205);
  motorSpeed = abs(motorSpeed);

  leftMotorSpeed = 50 + motorSpeed + steeringValue; // 50 + (0-205) + (0 - 205 ) =  50 - 255 so this range from 50 to 255 is used as PWM value
  rightMotorSpeed = 50 + motorSpeed - steeringValue;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255); // constrain the PWM value from 0 to 255
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // if PWM is lower than 52, set PWM value to 0

  if (leftMotorSpeed < 52) {
    leftMotorSpeed = 0;
  }
  if (rightMotorSpeed < 52) {
    rightMotorSpeed = 0;
  }


  // if right joystick goes up > move forward
  if (ch1 > 1510 && ch1 <= 2000) {
    digitalWrite(M1_RPWM, 0);
    analogWrite(M1_LPWM, leftMotorSpeed);

    digitalWrite(M2_RPWM, 0);
    analogWrite(M2_LPWM, rightMotorSpeed);
  }
  // if right joystick goes down > move backward
  if (ch1 >= 1000 && ch1 < 1490) {
    analogWrite(M1_RPWM, leftMotorSpeed);
    digitalWrite(M1_LPWM, 0);

    analogWrite(M2_RPWM, rightMotorSpeed);
    digitalWrite(M2_LPWM, 0);

  }

  // if right joystick is in the middle, don't move
  if (ch1 >= 1490 && ch1 <= 1510) {
    if (leftMotorSpeed < 50 && rightMotorSpeed < 50) {
      digitalWrite(M1_LPWM, 0);
      digitalWrite(M1_RPWM, 0);

      digitalWrite(M2_LPWM, 0);
      digitalWrite(M2_RPWM, 0);
    }
    // if right joystick move just left or right, without going up or down, move the tank left or right (in place)
    else if (ch0 < 1490 || ch0 > 1510) {
      analogWrite(M1_RPWM, rightMotorSpeed);
      analogWrite(M1_LPWM, leftMotorSpeed);

      analogWrite(M2_RPWM, leftMotorSpeed);
      analogWrite(M2_LPWM, rightMotorSpeed);


    }
  }
  //======= Gear shift ====
  if (ch6 == 2000) {
    myservo.write(140);
  }
  if (ch6 == 1000) {
    myservo.write(40);
  }

  //===== LIGHTS Control ===

  // Front and back lights LEDs control
  if (ch9 == 2000) {
    frontLeds[0] = CRGB (255, 255, 255);
    frontLeds[1] = CRGB (255, 255, 255);
    frontLeds[2] = CRGB (255, 255, 255);
    frontLeds[3] = CRGB (255, 255, 255);
    frontLeds[4] = CRGB (255, 255, 255);
    frontLeds[5] = CRGB (0, 0, 0);
    frontLeds[6] = CRGB (0, 0, 0);
    frontLeds[7] = CRGB (0, 0, 0);
    frontLeds[8] = CRGB (0, 0, 0);
    frontLeds[9] = CRGB (0, 0, 0);
    frontLeds[10] = CRGB (0, 0, 0);
    frontLeds[11] = CRGB (255, 255, 255);
    frontLeds[12] = CRGB (255, 255, 255);
    frontLeds[13] = CRGB (255, 255, 255);
    frontLeds[14] = CRGB (255, 255, 255);
    frontLeds[15] = CRGB (255, 255, 255);
    backLeftLeds[0] = CRGB ( 255, 0, 0);
    backLeftLeds[1] = CRGB ( 255, 0, 0);
    backLeftLeds[2] = CRGB ( 255, 0, 0);
    backRightLeds[0] = CRGB ( 255, 0, 0);
    backRightLeds[1] = CRGB ( 255, 0, 0);
    backRightLeds[2] = CRGB ( 255, 0, 0);
  }
  if (ch9 != ch9State) {
    if (ch9 == 1000) {
      FastLED.clear();
    }
    ch9State = ch9;
    FastLED.show();
  }

  // Front and back lights LEDs control
  if (ch8 == 1500) {
    frontLeds[0] = CRGB (255, 255, 255);
    frontLeds[1] = CRGB (255, 255, 255);
    frontLeds[2] = CRGB (255, 255, 255);
    frontLeds[3] = CRGB (255, 255, 255);
    frontLeds[4] = CRGB (255, 255, 255);
    frontLeds[5] = CRGB (255, 255, 255);
    frontLeds[6] = CRGB (255, 255, 255);
    frontLeds[7] = CRGB (255, 255, 255);
    frontLeds[8] = CRGB (255, 255, 255);
    frontLeds[9] = CRGB (255, 255, 255);
    frontLeds[10] = CRGB (255, 255, 255);
    frontLeds[11] = CRGB (255, 255, 255);
    frontLeds[12] = CRGB (255, 255, 255);
    frontLeds[13] = CRGB (255, 255, 255);
    frontLeds[14] = CRGB (255, 255, 255);
    frontLeds[15] = CRGB (255, 255, 255);
    backLeftLeds[0] = CRGB ( 255, 0, 0);
    backLeftLeds[1] = CRGB ( 255, 0, 0);
    backLeftLeds[2] = CRGB ( 255, 0, 0);
    backRightLeds[0] = CRGB ( 255, 0, 0);
    backRightLeds[1] = CRGB ( 255, 0, 0);
    backRightLeds[2] = CRGB ( 255, 0, 0);
  }
  if (ch8 == 2000) {
    frontLeds[0] = CRGB (255, 255, 255);
    frontLeds[1] = CRGB (255, 255, 255);
    frontLeds[2] = CRGB (255, 255, 255);
    frontLeds[3] = CRGB (255, 255, 255);
    frontLeds[4] = CRGB (255, 255, 255);
    frontLeds[5] = CRGB (255, 255, 255);
    frontLeds[6] = CRGB (255, 255, 255);
    frontLeds[7] = CRGB (255, 255, 255);
    frontLeds[8] = CRGB (255, 255, 255);
    frontLeds[9] = CRGB (255, 255, 255);
    frontLeds[10] = CRGB (255, 255, 255);
    frontLeds[11] = CRGB (255, 255, 255);
    frontLeds[12] = CRGB (255, 255, 255);
    frontLeds[13] = CRGB (255, 255, 255);
    frontLeds[14] = CRGB (255, 255, 255);
    frontLeds[15] = CRGB (255, 255, 255);
    backLeftLeds[0] = CRGB ( 255, 0, 0);
    backLeftLeds[1] = CRGB ( 255, 0, 0);
    backLeftLeds[2] = CRGB ( 255, 0, 0);
    backRightLeds[0] = CRGB ( 255, 0, 0);
    backRightLeds[1] = CRGB ( 255, 0, 0);
    backRightLeds[2] = CRGB ( 255, 0, 0);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 40) {

      if (i1 == 0) {
        i1 = 1;
      }
      else if (i1 == 1) {
        i1 = 0;
      }
      previousMillis = currentMillis;
    }

    if (i1 == 0) {
      FastLED.clear ();
      //FastLED.show ();
    }
    if (i1 == 1) {
      //FastLED.show ();
    }
    if (i1 != i1State) {
      i1State = i1;
      FastLED.show();
    }
  }

  if (ch8 != ch8State) {
    if (ch8 == 1000) {
      FastLED.clear();
    }
    ch8State = ch8;
    FastLED.show();
  }

  Serial.print("   i1:");
  Serial.println(i1);
  Serial.print("   ch8:");
  Serial.print(ch8);


  //======= Police strobe

  if (ch7 > 1500) {

    if (r == 4) {
      interval = 110;
    }
    else if (r == 6) {
      interval = 120;
    }
    else if (r == 8) {
      interval = 5;
    }
    else {
      interval = 60;
    }
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

      if (i1 == 0) {
        i1 = 1;
      }
      else if (i1 == 1) {
        i1 = 0;
        k++;
      }
      previousMillis = currentMillis;
    }


    if (i1 == 0) {
      if (r == 4) {
        // do nothing
      }
      else if (r == 6) {
        // do nothing
      }
      else if (r == 8) {
        // do nothing
      }
      else {
        FastLED.clear ();
      }
    }
    else if (i1 == 1) {
      switch (r) {
        case 1:
          switch (k) {
            case 1:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);

              //FastLED.show();
              break;
            case 2:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              //FastLED.show();
              break;
            case 3:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              //FastLED.show();
              break;
            case 4:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 5:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 6:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              j++;
              if (j < 3) {
                k = 0;
              }
              break;
            case 7:
              r = 2;
              k = 1;
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);
              break;
          }

          break;

        case 2:
          frontLeds[0] = CRGB (255, 0, 0);
          frontLeds[1] = CRGB (255, 0, 0);
          frontLeds[2] = CRGB (255, 0, 0);
          frontLeds[3] = CRGB (255, 0, 0);
          frontLeds[4] = CRGB (255, 0, 0);
          frontLeds[5] = CRGB (255, 0, 0);
          frontLeds[6] = CRGB (255, 255, 255);
          frontLeds[7] = CRGB (255, 255, 255);
          frontLeds[8] = CRGB (255, 255, 255);
          frontLeds[9] = CRGB (255, 255, 255);
          frontLeds[10] = CRGB (0, 0, 255);
          frontLeds[11] = CRGB (0, 0, 255);
          frontLeds[12] = CRGB (0, 0, 255);
          frontLeds[13] = CRGB (0, 0, 255);
          frontLeds[14] = CRGB (0, 0, 255);
          frontLeds[15] = CRGB (0, 0, 255);

          backLeftLeds[0] = CRGB ( 255, 0, 0);
          backLeftLeds[1] = CRGB ( 255, 0, 0);
          backLeftLeds[2] = CRGB ( 255, 0, 0);
          backRightLeds[0] = CRGB (0, 0, 255);
          backRightLeds[1] = CRGB (0, 0, 255);
          backRightLeds[2] = CRGB (0, 0, 255);
          if (k == 10) {
            k = 1;
            r = 3;
            j = 0;
          }
          break;
        case 3:
          switch (k) {
            case 1:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 2:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 3:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 4:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 5:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 6:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();

              j++;
              if (j < 2) {
                k = 0;
                //r = 1;
                //j = 0;
              }
              break;
            case 7:
              r = 4;
              k = 0;
              k2 = 0;
              j = 0;
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (0, 0, 0);
              frontLeds[5] = CRGB (0, 0, 0);
              frontLeds[6] = CRGB (0, 0, 0);
              frontLeds[7] = CRGB (0, 0, 0);
              frontLeds[8] = CRGB (0, 0, 0);
              frontLeds[9] = CRGB (0, 0, 0);
              frontLeds[10] = CRGB (0, 0, 0);
              frontLeds[11] = CRGB (0, 0, 0);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);
              break;
          }
          break;
        case 4: // 4th
          //---------
          switch (k) {
            case 1:
              frontLeds[0] = CRGB (0, 0, 0);
              frontLeds[1] = CRGB (0, 0, 0);
              frontLeds[2] = CRGB (0, 0, 0);
              frontLeds[3] = CRGB (0, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);
              frontLeds[6] = CRGB (255, 0, 0);
              frontLeds[7] = CRGB (255, 0, 0);
              frontLeds[8] = CRGB (0, 0, 255);
              frontLeds[9] = CRGB (0, 0, 255);
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 0);
              frontLeds[13] = CRGB (0, 0, 0);
              frontLeds[14] = CRGB (0, 0, 0);
              frontLeds[15] = CRGB (0, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              break;
            case 2:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (0, 0, 0);
              frontLeds[5] = CRGB (0, 0, 0);
              frontLeds[6] = CRGB (0, 0, 0);
              frontLeds[7] = CRGB (0, 0, 0);
              frontLeds[8] = CRGB (0, 0, 0);
              frontLeds[9] = CRGB (0, 0, 0);
              frontLeds[10] = CRGB (0, 0, 0);
              frontLeds[11] = CRGB (0, 0, 0);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backLeftLeds[0] = CRGB (0, 0, 255);
              backLeftLeds[1] = CRGB (0, 0, 255);
              backLeftLeds[2] = CRGB (0, 0, 255);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              j++;
              if (j < 8) {
                k = 0;
              }
              break;
            case 3:
              r = 5;
              k = 1;
              j = 0;
              break;
          }

          break; //--------- End of 4th light show

        case 5: // ---- 5th light show
          switch (k) {
            case 1:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 2:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 3:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 4:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 5:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 6:
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              break;
            case 7:
              r = 6;
              k = 1;
              j = 0;
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (0, 0, 0);
              frontLeds[5] = CRGB (0, 0, 0);
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);
              frontLeds[10] = CRGB (0, 0, 0);
              frontLeds[11] = CRGB (0, 0, 0);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);
              break;
          }
          break; //--------- End of 5th light show

        case 6: // ---- 6th light show
          switch (k) {
            case 1:
              frontLeds[0] = CRGB (0, 0, 0);
              frontLeds[1] = CRGB (0, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);
              frontLeds[6] = CRGB (0, 0, 0);
              frontLeds[7] = CRGB (0, 0, 0);
              frontLeds[8] = CRGB (0, 0, 0);
              frontLeds[9] = CRGB (0, 0, 0);
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 0);
              frontLeds[15] = CRGB (0, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              break;
            case 2:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (0, 0, 0);
              frontLeds[5] = CRGB (0, 0, 0);
              frontLeds[6] = CRGB (255, 255, 255);
              frontLeds[7] = CRGB (255, 255, 255);
              frontLeds[8] = CRGB (255, 255, 255);
              frontLeds[9] = CRGB (255, 255, 255);
              frontLeds[10] = CRGB (0, 0, 0);
              frontLeds[11] = CRGB (0, 0, 0);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backLeftLeds[0] = CRGB (0, 0, 255);
              backLeftLeds[1] = CRGB (0, 0, 255);
              backLeftLeds[2] = CRGB (0, 0, 255);
              backRightLeds[0] = CRGB ( 255, 0, 0);
              backRightLeds[1] = CRGB ( 255, 0, 0);
              backRightLeds[2] = CRGB ( 255, 0, 0);
              j++;
              if (j < 8) {
                k = 0;
              }
              break;
            case 3:
              r = 7;
              k = 1;
              j = 0;
              frontLeds[0] = CRGB (0, 0, 255);
              frontLeds[1] = CRGB (0, 0, 255);
              frontLeds[2] = CRGB (0, 0, 255);
              frontLeds[3] = CRGB (0, 0, 255);
              frontLeds[4] = CRGB (0, 0, 255);
              frontLeds[5] = CRGB (0, 0, 255);
              frontLeds[6] = CRGB (0, 0, 255);
              frontLeds[7] = CRGB (0, 0, 255);
              frontLeds[8] = CRGB (0, 0, 255);
              frontLeds[9] = CRGB (0, 0, 255);
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);
              break;
          }
          break; //--------- End of 6th light show

        case 7:// ---- 7th light show
          switch (k) {
            case 1:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              //FastLED.show();
              break;
            case 2:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              //FastLED.show();
              break;
            case 3:
              frontLeds[0] = CRGB (255, 0, 0);
              frontLeds[1] = CRGB (255, 0, 0);
              frontLeds[2] = CRGB (255, 0, 0);
              frontLeds[3] = CRGB (255, 0, 0);
              frontLeds[4] = CRGB (255, 0, 0);
              frontLeds[5] = CRGB (255, 0, 0);

              backLeftLeds[0] = CRGB ( 255, 0, 0);
              backLeftLeds[1] = CRGB ( 255, 0, 0);
              backLeftLeds[2] = CRGB ( 255, 0, 0);
              //FastLED.show();
              break;
            case 4:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 5:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();
              break;
            case 6:
              frontLeds[10] = CRGB (0, 0, 255);
              frontLeds[11] = CRGB (0, 0, 255);
              frontLeds[12] = CRGB (0, 0, 255);
              frontLeds[13] = CRGB (0, 0, 255);
              frontLeds[14] = CRGB (0, 0, 255);
              frontLeds[15] = CRGB (0, 0, 255);

              backRightLeds[0] = CRGB (0, 0, 255);
              backRightLeds[1] = CRGB (0, 0, 255);
              backRightLeds[2] = CRGB (0, 0, 255);
              //FastLED.show();

              j++;
              if (j < 2) {
                k = 0;
                //r = 1;
                //j = 0;
              }
              break;
            case 7:
              r = 8;
              k = 0;
              k2 = 0;
              j = 0;
              break;
          }
          break; //--------- End of 7th light show

        case 8: // ---- 8th light show
          if (k < 16) {
            frontLeds[k] = CRGB (255, 0, 0);

            backLeftLeds[0] = CRGB ( 255, 0, 0);
            backLeftLeds[1] = CRGB ( 255, 0, 0);
            backLeftLeds[2] = CRGB ( 255, 0, 0);
            backRightLeds[0] = CRGB (0, 0, 255);
            backRightLeds[1] = CRGB (0, 0, 255);
            backRightLeds[2] = CRGB (0, 0, 255);
            k2 = k;
          }
          if (k >= 16) {

            frontLeds[k2] = CRGB (0, 0, 255);

            backLeftLeds[0] = CRGB ( 0, 0, 0);
            backLeftLeds[1] = CRGB ( 0, 0, 0);
            backLeftLeds[2] = CRGB ( 0, 0, 0);
            backRightLeds[0] = CRGB (0, 0, 0);
            backRightLeds[1] = CRGB (0, 0, 0);
            backRightLeds[2] = CRGB (0, 0, 0);
            k2 = (16 - (k - 16));
          }
          if (k2 == -1) {
            k = 0;
            j++;
            if (j >= 10) {
              j = 0;
              r = 1;
            }
          }

          break; //--------- End of 8th light show
      }
    }

    if (i1 != i1State) {
      i1State = i1;
      FastLED.show();
    }
  }
  // Monitor the battery voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.00 / 1023.00) * 3.02; // Convert the reading values from 5v to suitable 12V
  // Send battery voltage value to transmitter
  IBusSensor.loop();
  IBusSensor.setSensorMeasurement(1, voltage * 100);
}
