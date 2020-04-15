#include <Wire.h>
#include "Nunchuk.h"

//START_CPP_LIB
//Start of constants.hpp********************************************************

#ifndef ROBOTARM_CONSTANTS_HPP
#define ROBOTARM_CONSTANTS_HPP

#define LEN_UNIT "cm"
#define ANGLE_UNIT "deg"//"°"
#define STAR_LINE_64 "****************************************************************"

#define EOL "\n"

constexpr float L1 = 11.2;
constexpr float L2 = 5.6;
constexpr float L3 = 3.2;

constexpr float ALPHA_MIN = 0.0f;
constexpr float ALPHA_MAX = 90.0f;
constexpr float BETA_MIN = 50.0f;
constexpr float BETA_MAX = 130.0f;
constexpr float GAMMA_MIN = -60.0f;
constexpr float GAMMA_MAX = 60.0f;
constexpr float DELTA_MIN = -90.0f;
constexpr float DELTA_MAX = 90.0f;
constexpr float EPSILON_MIN = 0.0f;
constexpr float EPSILON_MAX = 180.0f;
constexpr float ZETA_MIN = 0.0f;
constexpr float ZETA_MAX = 180.0f;

constexpr float OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1;
constexpr float OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX;

#endif //ROBOTARM_CONSTANTS_HPP
//End of constants.hpp*********************************************************
//Start of coupling.hpp*********************************************************

#ifndef ROBOTARM_COUPLING_HPP
#define ROBOTARM_COUPLING_HPP

class Coupling {
  public:
  Coupling(float axleDistance,
  float couplerLength,
  float jointRadius,
  float servoHornRadius,
  float servoOffsetAngle,
  float jointOffsetAngle);

  Coupling(float axleDistance,
  float couplerLength,
  float jointRadius,
  float servoHornRadius,
  float servoOffsetAngle);

  /**
  * @param jointAngle between -x and +x
  * @return servoAngle between -90° and +90°
  */
  float
  getServoAngle(float jointAngle, bool optimizedMethod = false);// todo find out which method is faster on arduino

  /**
  * @warning this method is not tested!!!
  */
  float getJointAngle(float servoAngle, bool optimizedMethod = false);

  float getAxleDistance() const;

  float getCouplerLength() const;

  float getServoHornRadius() const;

  float getJointRadius() const;

  float getServoOffsetAngle() const;

  float getJointOffsetAngle() const;

  private:
  float d;
  float c;
  float s;
  float g;
  float servoOffsetAngle;
  float jointOffsetAngle;
};

#endif //ROBOTARM_COUPLING_HPP
//End of coupling.hpp**********************************************************
//Start of Point3d.hpp**********************************************************
#ifndef ROBOTARM_POINT3D_H
#define ROBOTARM_POINT3D_H


#include <string>

class Point3d {
  public:
  float x, y, z;

  Point3d();

  Point3d(float x, float y, float z);

  static float distance_between(Point3d *p1, Point3d *p2);

  float distance_to(Point3d *other);

  /**
  *     distance
  *   |<---------->|
  * start--------return----------target
  */
  static Point3d *in_direction(Point3d *start, Point3d *target, float distance);

  std::string toString();
};

#endif //ROBOTARM_POINT3D_H
//End of Point3d.hpp***********************************************************
//Start of Point3dLinkNode.hpp**************************************************

#ifndef ROBOTARM_POINT3DLINKNODE_H
#define ROBOTARM_POINT3DLINKNODE_H



class Point3dLinkNode : public Point3d {
  public:
  Point3dLinkNode *last, *next;

  Point3dLinkNode();

  explicit Point3dLinkNode(Point3d *from);
};


#endif //ROBOTARM_POINT3DLINKNODE_H
//End of Point3dLinkNode.hpp***************************************************
//Start of ramp3d.hpp***********************************************************

#ifndef ROBOTARM_RAMP3D_HPP
#define ROBOTARM_RAMP3D_HPP


class Ramp3d {
  public:
  double acceleration = 60; // unit/second^2
  double max_speed = 100; // unit/second
  double steps_per_second = 50;

  Ramp3d();

  void calculate_nonlinear(Point3d *start, Point3d *stop);

  void calculate_linear(Point3d *start, Point3d *stop, float desiredStepLength);

  public:
  Point3dLinkNode *getStartNode() const;

  Point3dLinkNode *getStopNode() const;

  private:

  Point3dLinkNode *startNode;
  Point3dLinkNode *stopNode;

  Point3dLinkNode *getStartRamp();
};

#endif //ROBOTARM_RAMP3D_HPP
//End of ramp3d.hpp************************************************************
//Start of Rotation3d.hpp*******************************************************

#ifndef ROBOTARM_ROTATION3D_HPP
#define ROBOTARM_ROTATION3D_HPP

class Rotation3d {
  public:
  float rotX, rotY, rotZ;

  /**
  * @param accX, accY, accZ in G or m/s^2
  */
  static Rotation3d fromAcceleration(float accX, float accY, float accZ);
};

#endif //ROBOTARM_ROTATION3D_HPP
//End of Rotation3d.hpp********************************************************
//Start of ServoState.hpp*******************************************************

#ifndef ROBOTARM_SERVOSTATE_HPP
#define ROBOTARM_SERVOSTATE_HPP


class ServoState {
  public:
  float alpha;//arm #1
  float beta;//arm #2
  float gamma;//arm #3
  float delta;//turntable
  float epsilon;//gripper rotation
  float zeta;//gripper fingers

  float u;//alpha joint to gamma joint angle
  float p2_x; // p2 is gamma joint
  float p2_y;

  ServoState();

  bool operator==(const ServoState &rhs) const;

  bool operator!=(const ServoState &rhs) const;

  void print();

  bool isValid();

  void updateCalculated(ServoState *from);
};


#endif //ROBOTARM_SERVOSTATE_HPP
//End of ServoState.hpp********************************************************
//Start of libRobotArm.hpp******************************************************

#ifndef ROBOTARM_LIBROBOTARM_HPP
#define ROBOTARM_LIBROBOTARM_HPP


class RobotArm {
  public:
  static ServoState internal_calc2d(float r, float z, float omega);

  static ServoState calc2d(float r, float z, float omega);

  static ServoState calc3d(float x, float y, float z, float omega);

  static void print_config();

  void goTo(Point3d *to, float omega);

  ServoState *getState() const;

  private:
  ServoState *state;
  static const float U_MAX;
};

#endif //ROBOTARM_LIBROBOTARM_HPP
//End of libRobotArm.hpp*******************************************************
//Start of coupling.cpp*********************************************************


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

Coupling::Coupling(float axleDistance,
float couplerLength,
float jointRadius,
float servoHornRadius,
float servoOffsetAngle)
: Coupling(axleDistance, couplerLength, jointRadius, servoHornRadius, servoOffsetAngle, servoOffsetAngle) {
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

float Coupling::getJointAngle(float servoAngle, bool optimizedMethod) {
  float gamma = servoOffsetAngle + 90 - servoAngle; // γ
  float b = sqrt(d * d + s * s - 2 * d * s * cos(radians(gamma)));
  float fr1 = (d * d + b * b - s * s) / (2 * d * b);
  float fr2 = (b * b + g * g - c * c) / (2 * b * g);
  float deltaRad;
  if (optimizedMethod) {
    deltaRad = acos(fr1 * fr2 - sqrt(1 - fr1 * fr1) * sqrt(1 - fr2 * fr2));
  } else {
    deltaRad = acos(fr1) + acos(fr2);
  }
  return degrees(deltaRad) + jointOffsetAngle - 90; // δ
}

float Coupling::getAxleDistance() const {
  return d;
}

float Coupling::getCouplerLength() const {
  return c;
}

float Coupling::getServoHornRadius() const {
  return s;
}

float Coupling::getJointRadius() const {
  return g;
}

float Coupling::getServoOffsetAngle() const {
  return servoOffsetAngle;
}

float Coupling::getJointOffsetAngle() const {
  return jointOffsetAngle;
}
//End of coupling.cpp**********************************************************
//Start of libRobotArm.cpp******************************************************



using namespace std;


ServoState RobotArm::internal_calc2d(float r, float z, float omega) {

  ServoState state;

  float x_gamma = cos(radians(omega)) * L3;
  float y_gamma = sin(radians(omega)) * L3;

  state.p2_x = r - x_gamma;
  state.p2_y = z + y_gamma;

  float c = sqrt(state.p2_x * state.p2_x + state.p2_y * state.p2_y);
  state.u = degrees(atan(state.p2_y / state.p2_x));

  state.alpha = state.u + degrees(acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)));
  state.beta = 180 - degrees(acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)));
  state.gamma = omega - state.alpha - state.beta;

  return state;
}


ServoState RobotArm::calc2d(float r, float z, float omega) {
  ServoState state = internal_calc2d(r, z, omega);
  if (state.isValid()) {
    return state;
  }

  // r and z are too far away
  float new_p2_x = cos(radians(state.u)) * U_MAX;
  float new_p2_y = sin(radians(state.u)) * U_MAX;

  r -= state.p2_x - new_p2_x;
  z -= state.p2_y - new_p2_y;

  return internal_calc2d(r, z, omega);
}

ServoState RobotArm::calc3d(float x, float y, float z, float omega) {
  float r = sqrt(x * x + y * y);
  ServoState state = calc2d(r, z, omega);
  state.delta = degrees(atan(y / x));
  return state;
}

void RobotArm::print_config() {
  Serial.print("***Robot Arm Config***\n");
  Serial.print("* L1=");
  Serial.print(L1);
  Serial.println(LEN_UNIT);

  Serial.print("* L2=");
  Serial.print(L2);
  Serial.println(LEN_UNIT);

  Serial.print("* L3=");
  Serial.print(L3);
  Serial.println(LEN_UNIT);

  Serial.print("* ");
  Serial.print(ALPHA_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... alpha ... ");
  Serial.print(ALPHA_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("* ");
  Serial.print(BETA_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... beta ... ");
  Serial.print(BETA_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("* ");
  Serial.print(GAMMA_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... gamma ... ");
  Serial.print(GAMMA_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("* ");
  Serial.print(DELTA_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... delta ... ");
  Serial.print(DELTA_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("* ");
  Serial.print(EPSILON_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... epsilon ... ");
  Serial.print(EPSILON_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("* ");
  Serial.print(ZETA_MIN);
  Serial.print(ANGLE_UNIT);
  Serial.print(" ... zeta ... ");
  Serial.print(ZETA_MAX);
  Serial.println(ANGLE_UNIT);

  Serial.print("**********************\n");
}

const float RobotArm::U_MAX = sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * cos(radians(180 - BETA_MIN)));// cosine law

ServoState *RobotArm::getState() const {
  return state;
}

void RobotArm::goTo(Point3d *to, float omega) {
  this->state->updateCalculated(new ServoState(RobotArm::calc3d(to->x, to->y, to->z, omega)));
}
//End of libRobotArm.cpp*******************************************************
//Start of Point3d.cpp**********************************************************
#include <sstream>


Point3d::Point3d() {
  x = y = z = 0.0f;
}

Point3d::Point3d(float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

float Point3d::distance_between(Point3d *p1, Point3d *p2) {
  float diffX = p1->x - p2->x;
  float diffY = p1->y - p2->y;
  float diffZ = p1->z - p2->z;
  return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

float Point3d::distance_to(Point3d *other) {
  return distance_between(this, other);
}

/**
*     distance
*   |<---------->|
* start--------return----------target
*/
Point3d *Point3d::in_direction(Point3d *start, Point3d *target, float distance) {
  float factor = distance / distance_between(start, target);
  float diffX = target->x - start->x;
  float diffY = target->y - start->y;
  float diffZ = target->z - start->z;
  return new Point3d(
  start->x + diffX * factor,
  start->y + diffY * factor,
  start->z + diffZ * factor);
}

std::string Point3d::toString() {
  std::stringstream result;
  result << "X=" << x << ", Y=" << y << ", Z=" << z;
  return result.str();
}
//End of Point3d.cpp***********************************************************
//Start of Point3dLinkNode.cpp**************************************************



Point3dLinkNode::Point3dLinkNode() {
  last = next = nullptr;
}

Point3dLinkNode::Point3dLinkNode(Point3d *from) {
  x = from->x;
  y = from->y;
  z = from->z;
  last = next = nullptr;
}
//End of Point3dLinkNode.cpp***************************************************
//Start of ramp3d.cpp***********************************************************

Ramp3d::Ramp3d() {
}

void Ramp3d::calculate_nonlinear(Point3d *start, Point3d *stop) {
  /*
  * start-------node_a   node_b-------stop
  */
  this->startNode = new Point3dLinkNode(start);
  this->stopNode = new Point3dLinkNode(stop);
  int i_step = 0;
  double total_distance = Point3d::distance_between(start, stop);
  double travelled_distance = 0;
  double travel_per_step = acceleration / steps_per_second;
  Point3dLinkNode *node_a = startNode;
  Point3dLinkNode *node_b = stopNode;
  while (travelled_distance * 2 + travel_per_step < total_distance) {
    Point3dLinkNode *new_a = new Point3dLinkNode(/*TODO*/);
  }
}

void Ramp3d::calculate_linear(Point3d *start, Point3d *stop, float desiredStepLength) {
  float total_distance = Point3d::distance_between(start, stop);
  this->startNode = new Point3dLinkNode(start);
  Point3dLinkNode *iNode = this->startNode;
  int numSteps = ceil(total_distance / desiredStepLength);
  float actualStepLength = total_distance / numSteps;
  float iDistance = 0;
  for (int i = 0; i < numSteps; ++i) {
    iDistance += actualStepLength;
    auto *newNode = new Point3dLinkNode(Point3d::in_direction(start, stop, iDistance));
    newNode->last = iNode;
    iNode->next = newNode;
    iNode = newNode;
  }
}

Point3dLinkNode *Ramp3d::getStartRamp() {
  //todo
  return nullptr;
}

Point3dLinkNode *Ramp3d::getStartNode() const {
  return startNode;
}

Point3dLinkNode *Ramp3d::getStopNode() const {
  return stopNode;
}



//End of ramp3d.cpp************************************************************
//Start of Rotation3d.cpp*******************************************************


Rotation3d Rotation3d::fromAcceleration(float accX, float accY, float accZ) {
  //todo check if implementation is correct
  // from https://forum.arduino.cc/index.php?topic=112031.0 Post #5
Rotation3d result{};

  result.rotX = degrees(atan2(-accY, -accZ)) + 180;
  result.rotY = degrees(atan2(-accX, -accZ)) + 180;
  result.rotZ = degrees(atan2(-accY, -accX)) + 180;
  return result;
}
//End of Rotation3d.cpp********************************************************
//Start of ServoState.cpp*******************************************************


using namespace std;


ServoState::ServoState() {
  alpha = 0.0f;
  beta = 0.0f;
  gamma = 0.0f;
  delta = 0.0f;
  epsilon = 0.0f;
  zeta = 0.0f;
  u = 0.0f;
  p2_x = 0.0f;
  p2_y = 0.0f;
}

bool ServoState::operator==(const ServoState &rhs) const {
  return alpha == rhs.alpha && beta == rhs.beta && gamma == rhs.gamma && delta == rhs.delta &&
  epsilon == rhs.epsilon && zeta == rhs.zeta && u == rhs.u && p2_x == rhs.p2_x && p2_y == rhs.p2_y;
}

bool ServoState::operator!=(const ServoState &rhs) const {
  return !(rhs == *this);
}

void ServoState::print() {
  Serial.print("***Servo state***\n");
  Serial.print("* alpha=");
  Serial.print(alpha);
  Serial.println(ANGLE_UNIT);

  Serial.print("* beta=");
  Serial.print(beta);
  Serial.println(ANGLE_UNIT);

  Serial.print("* gamma=");
  Serial.print(gamma);
  Serial.println(ANGLE_UNIT);

  Serial.print("* delta=");
  Serial.print(delta);
  Serial.println(ANGLE_UNIT);

  Serial.print("* epsilon=");
  Serial.print(epsilon);
  Serial.println(ANGLE_UNIT);

  Serial.print("* zeta=");
  Serial.print(zeta);
  Serial.println(ANGLE_UNIT);

  Serial.print("> u=");
  Serial.print(u);
  Serial.println(ANGLE_UNIT);

  Serial.print("> p2_x=");
  Serial.print(p2_x);
  Serial.println(LEN_UNIT);

  Serial.print("> p2_y=");
  Serial.print(p2_y);
  Serial.println(LEN_UNIT);

  Serial.print("*****************\n");
}

bool ServoState::isValid() {
  return !(
  isnan(alpha) || ALPHA_MIN > alpha || ALPHA_MAX < alpha ||
  isnan(beta) || BETA_MIN > beta || BETA_MAX < beta ||
  isnan(gamma) || GAMMA_MIN > gamma || GAMMA_MAX < gamma ||
  isnan(delta) || DELTA_MIN > delta || DELTA_MAX < delta ||
  isnan(epsilon) || EPSILON_MIN > epsilon || EPSILON_MAX < epsilon ||
  isnan(zeta) || ZETA_MIN > zeta || ZETA_MAX < zeta
  );
}

void ServoState::updateCalculated(ServoState *from) {
  this->alpha = from->alpha;
  this->beta = from->beta;
  this->gamma = from->gamma;
  this->delta = from->delta;
}
//End of ServoState.cpp********************************************************
//END_CPP_LIB


class HardwareController {
public:
    void initialize();
    RobotArm * getArm();
    void updateServos();
    void moveArm(float deltaX, float deltaY, float deltaZ);

    float absGripperAngle, gripperRotation;
private:
    const int NUM_SERVOS = 6;
    const int TURNTABLE = 0;
    const int JOINT_A = 1;
    const int JOINT_B = 2;
    const int JOINT_C = 3;
    const int GRIPPER_TURN = 4;
    const int GRIPPER_OPEN = 5;
    const int SERVO_PINS[] = {99, 98, 97, 96, 95, 94}; // todo look up correct pin numbers

    Servo[NUM_SERVOS] servos;
    RobotArm *arm = nullptr;
    float posX, posY, posZ;

    void attachAllServos();
};

void HardwareController::initialize() {
    arm = new RobotArm();
    attachAllServos();
}

void HardwareController::attachAllServos() {
    for (int i = 0; i < NUM_SERVOS; ++i) {
        servos[i] = Servo(SERVO_PINS[i]);
        servos[i].attach();
        servos[i].write(90);
    }
}
RobotArm * HardwareController::getArm() {
    return &arm;
}

void HardwareController::updateServos() {
    Point3d * tmpPoint = new Point3d(posX, posY, posZ);
    this->arm->goTo(tmpPoint, absGripperAngle);
    ServoState * state = this->arm->getState();
    servos[JOINT_A].write(state->alpha);
    servos[JOINT_B].write(state->beta);
    servos[JOINT_C].write(state->gamma);
    servos[TURNTABLE].write(state->delta);
    servos[GRIPPER_TURN].write(state->epsilon);
    servos[GRIPPER_OPEN].write(state->zeta);
    delete tmpPoint;
}

void HardwareController::moveArm(float deltaX, float deltaY, float deltaZ) {
    posX += deltaX;
    posY += deltaY;
    posZ += deltaZ;
}

HardwareController controller{};

const float NUNCHUK_G_PER_COUNT = 0.0188f;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  nunchuk_init();
  controller.initialize();
}

/**
 * Move X:                 Joystick left/right
 * Move Y:                 Joystick up/down
 * Move Z:                 Button C/Z
 * Rotate Gripper:         Tilt left/right
 * Absolute Gripper angle: Tilt forward/backward
 */
void loop() {
  if (nunchuk_read()) {
    //nunchuk_print();

    float x = nunchuk_joystickX();
    float y = nunchuk_joystickY();
    int c = nunchuk_buttonC();
    int z = nunchuk_buttonZ();

    Rotation3d rotation = Rotation3d::fromAcceleration(
            NUNCHUK_G_PER_COUNT * nunchuk_accelerationX(),
            NUNCHUK_G_PER_COUNT * nunchuk_accelerationY(),
            NUNCHUK_G_PER_COUNT * nunchuk_accelerationZ()
            );

    ServoState * state = controller.getArm()->getState();
    float omega = rotation.rotX;
    state->epsilon = rotation.rotY;
    controller.moveArm(x/100, y/100, c?0.2:z?-0.2:0);
    controller.updateServos();
    state->print();
  }
  delay(10);
}
