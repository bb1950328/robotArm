#define USE_COUPLING

#ifndef NO_HARDWARE
#include <Wire.h>
#include "Nunchuk.h"
#endif

#include <Servo.h>

#define LEN_UNIT "mm"
#define ANGLE_UNIT "deg"//"°"
#define STAR_LINE_64 "****************************************************************"

#define EOL "\n"

class Coupling {
public:
    Coupling(float axleDistance,
             float couplerLength,
             float jointRadius,
             float servoHornRadius,
             float servoOffsetAngle,
             float jointOffsetAngle);

    /**
    * @param jointAngle between -x and +x
    * @return servoAngle between -90° and +90°
    */
    float getServoAngle(float jointAngle, bool optimizedMethod = false);

private:
    float d;
    float c;
    float s;
    float g;
    float servoOffsetAngle;
    float jointOffsetAngle;
};

Coupling::Coupling(float axleDistance,
                   float couplerLength,
                   float jointRadius,
                   float servoHornRadius,
                   float servoOffsetAngle,
                   float jointOffsetAngle) {
    d = axleDistance;
    c = couplerLength;
    g = jointRadius;
    s = servoHornRadius;
    this->servoOffsetAngle = servoOffsetAngle;
    this->jointOffsetAngle = jointOffsetAngle;
}

float Coupling::getServoAngle(float jointAngle, bool optimizedMethod) {
    float delta = 90 - jointOffsetAngle + jointAngle;
    float a = sqrt(d * d + g * g - 2 * d * g * cos(radians(delta)));
    float fr1 = (d * d + a * a - g * g) / (2 * d * a);
    float fr2 = (a * a + s * s - c * c) / (2 * a * s);
    float lambdaRad;
    if (optimizedMethod) {
        lambdaRad = acos(fr1 * fr2 - sqrt(1 - fr1 * fr1) * sqrt(1 - fr2 * fr2));
    } else {
        lambdaRad = acos(fr1) + acos(fr2);
    }
    return servoOffsetAngle - degrees(lambdaRad) + 90;
}

/**--------------given------------- | --------------------------------calculated-----------------------------------|
    *  horn radius | distance | range  | servo horn offs | joint offs adj  | joint radius | connector | dx     | dy   |
    * a 17mm       | 94mm     | +/-45° | 10.41°          | -1.8°           | 24.04mm      | 92.98mm   | 92.5mm | 17mm |
    * b 28mm       | 77mm     | +/-40° | 25.6813°        | -3.85°          | 43.5603mm    | 71.6426mm | 69.4mm | 33mm |
    * c 30mm       | 125mm    | +/-50° | 11.6179°        | -2.4°           | 39.1622mm    | 123.481mm | 122mm  | 25mm |
    */


constexpr float L1 = 300;
constexpr float L2 = 200;
constexpr float L3 = 175;

constexpr float ALPHA_MIN = 0.0f;
constexpr float ALPHA_MAX = 90.0f;
constexpr float BETA_MIN = 50.0f;
constexpr float BETA_MAX = 130.0f;
constexpr float GAMMA_MIN = -50.0f;
constexpr float GAMMA_MAX = 50.0f;
constexpr float DELTA_MIN = -90.0f;
constexpr float DELTA_MAX = 90.0f;
constexpr float EPSILON_MIN = 0.0f;
constexpr float EPSILON_MAX = 180.0f;
constexpr float ZETA_MIN = 0.0f;
constexpr float ZETA_MAX = 180.0f;

constexpr float OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1;
constexpr float OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX;

const int JOINT_A = 0;
const int JOINT_B = 1;
const int JOINT_C = 2;
const int TURNTABLE = 3;
const int GRIPPER_TURN = 4;
const int GRIPPER_OPEN = 5;

const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11}; // todo look up correct pin numbers
const int NUM_SERVOS = 6;

const int BUTTON_OPEN_GRIPPER_PIN = -1;
const int BUTTON_CLOSE_GRIPPER_PIN = -1;
const int BUTTON_TURN_LEFT_PIN = -1;
const int BUTTON_TURN_RIGHT_PIN = -1;
const int BUTTON_ANGLE_LOWER_PIN = -1;
const int BUTTON_ANGLE_HIGHER_PIN = -1;

Servo servos[6];
float posX = 540;//mm
float posY = 0;//mm
float posZ = 40;//mm
float omega = 0;//deg
float gripperRotation = 90;//deg
float zeta = 90;//deg
float alpha, beta, gamma, delta;

#ifdef USE_COUPLING
Coupling *couplingA;
Coupling *couplingB;
Coupling *couplingC;
#endif

void attachAllServos() {
    for (int i = 0; i < NUM_SERVOS; ++i) {
        servos[i].attach(SERVO_PINS[i]);
        servos[i].write(90);
    }
}

float isValid() {
    return !(
            isnan(alpha) || ALPHA_MIN > alpha || ALPHA_MAX < alpha ||
            isnan(beta) || BETA_MIN > beta || BETA_MAX < beta ||
            isnan(gamma) || GAMMA_MIN > gamma || GAMMA_MAX < gamma ||
            isnan(delta) || DELTA_MIN > delta || DELTA_MAX < delta
    );
}

void calc2d(float r, float z, float omega) {
    long start = micros();
    float x_gamma = cos(radians(omega)) * L3;
    float y_gamma = sin(radians(omega)) * L3;

    float p2_x = r - x_gamma;
    float p2_y = z + y_gamma;

    float c = sqrt(p2_x * p2_x + p2_y * p2_y);
    float u = degrees(atan(p2_y / p2_x));

    alpha = u + degrees(acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)));
    beta = 180 - degrees(acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)));
    gamma = omega + (alpha - beta);
    long end = micros();

    Serial.print("Used");
    Serial.print(end - start);
    Serial.println("us");

    Serial.print("r: ");
    Serial.println(r);

    Serial.print("z: ");
    Serial.println(z);

    Serial.print("u: ");
    Serial.println(u);

    Serial.print("alpha=");
    Serial.print(alpha);
    Serial.println(ANGLE_UNIT);

    Serial.print("beta=");
    Serial.print(beta);
    Serial.println(ANGLE_UNIT);

    Serial.print("gamma=");
    Serial.print(gamma);
    Serial.println(ANGLE_UNIT);

    Serial.print("delta=");
    Serial.print(delta);
    Serial.println(ANGLE_UNIT);
}

void calc3d() {
    float r = sqrt(posX * posX + posY * posY);
    calc2d(r, posZ, omega);
    delta = degrees(atan(posY / posX));
}


void updateServos() {
#ifdef USE_COUPLING
    float correctAlpha = couplingA->getServoAngle(alpha);
    float correctBeta = couplingB->getServoAngle(beta);
    float correctGamma = couplingC->getServoAngle(gamma);
    Serial.print("Correct servo angles: ");
    Serial.print(correctAlpha);
    Serial.print(";");
    Serial.print(correctBeta);
    Serial.print(";");
    Serial.println(correctGamma);

    servos[JOINT_A].write(correctAlpha);
    servos[JOINT_B].write(correctBeta);
    servos[JOINT_C].write(correctGamma);
#else
    servos[JOINT_A].write(map(alpha, ALPHA_MIN, ALPHA_MAX, 0, 180));
    servos[JOINT_B].write(map(beta, BETA_MIN, BETA_MAX, 0, 180));
    servos[JOINT_C].write(map(gamma, GAMMA_MIN, GAMMA_MAX, 0, 180));
#endif
    servos[TURNTABLE].write(map(delta, DELTA_MIN, DELTA_MAX, 0, 180));
    servos[GRIPPER_TURN].write(gripperRotation);
    servos[GRIPPER_OPEN].write(zeta);
}

#ifndef NO_HARDWARE
void refreshValues() {
    float x = nunchuk_joystickX();
    float y = nunchuk_joystickY();
    int c = nunchuk_buttonC();
    int z = nunchuk_buttonZ();

    if (!digitalRead(BUTTON_ANGLE_HIGHER_PIN)) {
        omega = min(45, omega + 1);
    } else if (!digitalRead(BUTTON_ANGLE_LOWER_PIN)) {
        omega = min(-45, omega - 1);
    }

    if (!digitalRead(BUTTON_TURN_LEFT_PIN)) {
        gripperRotation = min(180, gripperRotation + 1);
    } else if (!digitalRead(BUTTON_TURN_RIGHT_PIN)) {
        gripperRotation = max(0, gripperRotation - 1);
    }

    if (!digitalRead(BUTTON_OPEN_GRIPPER_PIN)) {
        zeta = min(180, zeta + 1);
    } else if (!digitalRead(BUTTON_CLOSE_GRIPPER_PIN)) {
        zeta = max(0, zeta - 1);
    }

    posX += (x / 100);
    posY += (y / 100);
    posZ += (c ? 0.2 : z ? -0.2 : 0);
}
#endif

void setup() {
    Serial.begin(9600);
    Serial.println("Start");
#ifndef NO_HARDWARE
    Wire.begin();
    nunchuk_init();
#endif
    attachAllServos();

    pinMode(BUTTON_OPEN_GRIPPER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CLOSE_GRIPPER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_TURN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_TURN_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ANGLE_LOWER_PIN, INPUT_PULLUP);
    pinMode(BUTTON_ANGLE_HIGHER_PIN, INPUT_PULLUP);
#ifdef USE_COUPLING
    couplingA = new Coupling(94, 92.9848, 24.0416, 17, 10.4193, 8.61934);
    couplingB = new Coupling(77, 71.6426, 43.5603, 28, 25.6813, 21.8313);
    couplingC = new Coupling(125, 123.481, 39.1622, 30, 11.6179, 9.2179);
#endif
}

/**
 * Move X:                 Joystick left/right
 * Move Y:                 Joystick up/down
 * Move Z:                 Button C/Z
 * Rotate Gripper:         Buttongroup 1
 * Absolute Gripper angle: Buttongroup 2
 * Grip:                   Buttongroup 3
 */
void loop() {

#ifndef NO_HARDWARE
    if (nunchuk_read()) {
        refreshValues();
#else
    if (true) {
        posX += 1;
#endif
        Serial.println("Starting calculation now.");
        calc3d();

        if (isValid()) {
            updateServos();
        } else {
            Serial.println("angles aren't valid!!!");
        }
    }
    delay(1000);
}
