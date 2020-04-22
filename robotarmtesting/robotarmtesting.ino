#include <Wire.h>
#include "Nunchuk.h"
#include <Servo.h>
#define ARDUINO

// define all Servos
Servo sa, sb, table, gripperLower, gripperTurn, gripperOpen;

// define all Buttonpins
const int BUTTON_OPEN_GRIPPER_PIN = 2;
const int BUTTON_CLOSE_GRIPPER_PIN = 4;
const int BUTTON_TURN_LEFT_PIN = 7;
const int BUTTON_TURN_RIGHT_PIN = 8;
const int BUTTON_ANGLE_LOWER_PIN = 12;
const int BUTTON_ANGLE_HIGHER_PIN = 13;

// define all min am max angles
const int MIN = 0;
const int MAX = 180;


// define all start angles from the servos
int angleA = 90;
int angleB = 90;
int tableW = 90;
int gripperLowerAngle = 90;
int gripperTurnAngle = 90;
int gripperOpenAngle = 90;

void setup() {
    // Servos den Pins zuweisen
    sa.attach(5);
    sb.attach(10);
    table.attach(3);
    gripperLower.attach(9);
    gripperTurn.attach(6);
    gripperOpen.attach(11);

    // Nunchuk einbinden
    Serial.begin(9600);
    Wire.begin();
    nunchuk_init();

    // Buttons als Inputs definieren
    pinMode(BUTTON_OPEN_GRIPPER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CLOSE_GRIPPER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_TURN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_TURN_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ANGLE_LOWER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ANGLE_HIGHER_PIN, INPUT_PULLUP);
}

void loop() {
  if (nunchuk_read()) {
    // Werte von Joystick auslesen
    float x = nunchuk_joystickX();
    float y = nunchuk_joystickY();
    int c = nunchuk_buttonC();
    int z = nunchuk_buttonZ();


    // Roboter drehen
    if(MIN < tableW) {
      if(x > 90) {
        tableW -= 1;
      } else if(x > 30) {
        tableW -= 0.5;
      }
    }

    if(MAX > tableW) {
      if(x < -90) {
        tableW += 1;
      } else if(x < -30) {
        tableW += 0.5;
      }
    }

    // Roboterarm vorne bewegen
    if(MIN < angleB) {
      if(y > 90) {
        angleB -= 1;
      } else if(y > 30) {
        angleB -= 0.5;
      }
    }

    if(MAX > angleB) {
      if(y < -90) {
        angleB += 1;
      } else if(y < -30) {
        angleB += 0.5;
      }
    }

    // Roboterarm hinten bewegen
    if(z == 1 && MIN < angleA) {
      angleA -= 1;
    }
    
    if(c == 1 && MAX > angleA) {
      angleA += 1;
    }

    // Greifer nach oben oder unten
    if (!digitalRead(BUTTON_ANGLE_HIGHER_PIN) && MAX > gripperLowerAngle) {
        gripperLowerAngle += 1;
    } else if (!digitalRead(BUTTON_ANGLE_LOWER_PIN) && MIN < gripperLowerAngle) {
        gripperLowerAngle -= 1;
    }

    // Greifer drehen
    if (!digitalRead(BUTTON_TURN_LEFT_PIN) && MAX > gripperTurnAngle) {
        gripperTurnAngle += 1;
    } else if (!digitalRead(BUTTON_TURN_RIGHT_PIN) && MIN < gripperTurnAngle) {
        gripperTurnAngle -= 1;
    }

    // greifen
    if (!digitalRead(BUTTON_OPEN_GRIPPER_PIN) && MAX > gripperOpenAngle) {
        gripperOpenAngle += 1;
    } else if (!digitalRead(BUTTON_CLOSE_GRIPPER_PIN) && MIN < gripperOpenAngle) {
        gripperOpenAngle -= 1;
    }
    
    // alle Servos bewegen
    sa.write(angleA);
    sb.write(angleB);
    table.write(tableW);
    gripperLower.write(gripperLowerAngle);
    gripperTurn.write(gripperTurnAngle);
    gripperOpen.write(gripperOpenAngle);
  }
  delay(10);
}
