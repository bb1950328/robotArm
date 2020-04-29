#include <Wire.h>
#include "Nunchuk.h"
#include <Servo.h>
#define ARDUINO

// Alle Servovariablen definiern
Servo sa, sb, table, gripperLower, gripperTurn, gripperOpen;

// Alle Buttonpins Definieren
const int BUTTON_OPEN_GRIPPER_PIN = 2;
const int BUTTON_CLOSE_GRIPPER_PIN = 4;
const int BUTTON_TURN_LEFT_PIN = 7;
const int BUTTON_TURN_RIGHT_PIN = 8;
const int BUTTON_ANGLE_LOWER_PIN = 12;
const int BUTTON_ANGLE_HIGHER_PIN = 13;

// Minimal- und Maximalwinkel definieren
const int MIN = 0;
const int MAX = 180;
const int MIN_A = 10;
const int MAX_A = 180;
const int MIN_B = 0;
const int MAX_B = 110;
const int MIN_W = 0;
const int MAX_W = 180;
const int MIN_GRIPPERL = 0;
const int MAX_GRIPPERL = 180;
const int MIN_GRIPPERT = 0;
const int MAX_GRIPPERT = 180;


// Startposition der Winkel definieren
float angleA = 90;
float angleB = 90;
float tableW = 90;
float gripperLowerAngle = 90;
float gripperTurnAngle = 90;
float gripperOpenAngle = 90;

void setup() {
    // Servos den Pins zuweisen
    sa.attach(3);  // Black
    sb.attach(5);  // Yellow
    table.attach(9);  // Brown
    gripperLower.attach(6);  // Orange
    gripperTurn.attach(10);  // Red
    gripperOpen.attach(11);  // Green

    // Serielleschnitstelle
    Serial.begin(9600);

    // Nunchuk einbinden
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

    Serial.println("PRINT:__________");
    Serial.println(nunchuk_buttonC());
    Serial.println(c);

    // Roboterarm drehen
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

    // Armteil vorne bewegen
    if(MIN_B < angleB) {
      if(y > 90) {
        angleB -= 1;
      } else if(y > 30) {
        angleB -= 0.5;
      }
    }

    if(MAX_B > angleB) {
      if(y < -90) {
        angleB += 1;
      } else if(y < -30) {
        angleB += 0.5;
      }
    }

    // Roboterarm nach oben und unten bewegen
    if(z == 1 && MIN_A < angleA) {
      angleA -= 0.3;
    }
    
    if(c == 1 && MAX_A > angleA) {
      angleA += 0.3;
    }

    // Greifer nach oben oder unten
    if (!digitalRead(BUTTON_ANGLE_HIGHER_PIN) && MAX_GRIPPERL > gripperLowerAngle) {
        gripperLowerAngle += 1;
    } else if (!digitalRead(BUTTON_ANGLE_LOWER_PIN) && MIN_GRIPPERL < gripperLowerAngle) {
        gripperLowerAngle -= 1;
    }

    // Greifer drehen
    if (!digitalRead(BUTTON_TURN_LEFT_PIN) && MAX_GRIPPERT > gripperTurnAngle) {
        gripperTurnAngle += 1;
    } else if (!digitalRead(BUTTON_TURN_RIGHT_PIN) && MIN_GRIPPERT < gripperTurnAngle) {
        gripperTurnAngle -= 1;
    }

    // greifen
    if (!digitalRead(BUTTON_OPEN_GRIPPER_PIN) && MAX > gripperOpenAngle) {
        gripperOpenAngle += 1;
    } else if (!digitalRead(BUTTON_CLOSE_GRIPPER_PIN) && MIN < gripperOpenAngle) {
        gripperOpenAngle -= 1;
    }
    
    // alle Servos beschreiben
    sa.write(angleA);
    sb.write(angleB);
    table.write(tableW);
    gripperLower.write(gripperLowerAngle);
    gripperTurn.write(gripperTurnAngle);
    gripperOpen.write(gripperOpenAngle);
  }
  delay(10);
}
