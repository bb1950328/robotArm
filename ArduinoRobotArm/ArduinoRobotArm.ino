//START_CPP_LIB
//Start of constants.hpp********************************************************

#ifndef ROBOTARM_CONSTANTS_HPP
#define ROBOTARM_CONSTANTS_HPP

#define LEN_UNIT "cm"
#define ANGLE_UNIT "deg."//"°"

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
float l;
float la, lb;

Coupling(float couplerLength, float jointRadius, float servoHornRadius);

float getServoAngle(float jointAngle);
};

#endif //ROBOTARM_COUPLING_HPP
//End of coupling.hpp**********************************************************
//Start of Point3d.hpp**********************************************************
#ifndef ROBOTARM_POINT3D_H
#define ROBOTARM_POINT3D_H


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


private:
Point3dLinkNode *startNode;
Point3dLinkNode *stopNode;

Point3dLinkNode *getStartRamp();

void calculate_nonlinear(Point3d *start, Point3d *stop);

void calculate_linear(Point3d *start, Point3d *stop, float desiredStepLength);
};
#endif //ROBOTARM_RAMP3D_HPP
//End of ramp3d.hpp************************************************************
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

private:
static const float U_MAX;
};

#endif //ROBOTARM_LIBROBOTARM_HPP
//End of libRobotArm.hpp*******************************************************
//Start of coupling.cpp*********************************************************

Coupling::Coupling(float couplerLength, float jointRadius, float servoHornRadius) {
this->l = couplerLength;
this->la = jointRadius;
this->lb = servoHornRadius;
}

float Coupling::getServoAngle(float jointAngle) {
float dx = sin(radians(jointAngle)) * la;
float dy = cos(radians(jointAngle)) * la;

float l_dx = l - dx;
float n = sqrt(dy * dy + l_dx * l_dx);

float gamma = degrees(acos((n * n + lb * lb - l * l) / (2 * n * lb)));
float delta = degrees(atan(dy / l_dx));
return delta + gamma - 90;
}
//End of coupling.cpp**********************************************************
//Start of libRobotArm.cpp******************************************************



using namespace std;


ServoState RobotArm::internal_calc2d(float r, float z, float omega) {

ServoState state;//asf

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
//cout << "***Robot Arm Config***\n";
//cout << "* L1=" << L1 << LEN_UNIT << EOL;
//cout << "* L2=" << L2 << LEN_UNIT << EOL;
//cout << "* L3=" << L3 << LEN_UNIT << EOL;
//cout << "* " << ALPHA_MIN << ANGLE_UNIT << " ... alpha ... " << ALPHA_MAX << ANGLE_UNIT << EOL;
//cout << "* " << BETA_MIN << ANGLE_UNIT << " ... beta ... " << BETA_MAX << ANGLE_UNIT << EOL;
//cout << "* " << GAMMA_MIN << ANGLE_UNIT << " ... gamma ... " << GAMMA_MAX << ANGLE_UNIT << EOL;
//cout << "* " << DELTA_MIN << ANGLE_UNIT << " ... delta ... " << DELTA_MAX << ANGLE_UNIT << EOL;
//cout << "* " << EPSILON_MIN << ANGLE_UNIT << " ... epsilon ... " << EPSILON_MAX << ANGLE_UNIT << EOL;
//cout << "* " << ZETA_MIN << ANGLE_UNIT << " ... zeta ... " << ZETA_MAX << ANGLE_UNIT << EOL;
//cout << "**********************\n";
}

const float RobotArm::U_MAX = sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * cos(radians(180 - BETA_MIN))); // cosine law
//End of libRobotArm.cpp*******************************************************
//Start of Point3d.cpp**********************************************************


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
float diffX = start->x - target->x;
float diffY = start->y - target->y;
float diffZ = start->z - target->z;
return new Point3d(
start->x + diffX * factor,
start->y + diffY * factor,
start->z + diffZ * factor);
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
float actualStepLength = total_distance/numSteps;
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



//End of ramp3d.cpp************************************************************
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
//cout << "***Servo state***\n";
//cout << "* alpha=" << alpha << ANGLE_UNIT << EOL;
//cout << "* beta=" << beta << ANGLE_UNIT << EOL;
//cout << "* gamma=" << gamma << ANGLE_UNIT << EOL;
//cout << "* delta=" << delta << ANGLE_UNIT << EOL;
//cout << "* epsilon=" << epsilon << ANGLE_UNIT << EOL;
//cout << "* zeta=" << zeta << ANGLE_UNIT << EOL;
//cout << "> u=" << u << ANGLE_UNIT << EOL;
//cout << "> p2_x=" << p2_x << LEN_UNIT << EOL;
//cout << "> p2_y=" << p2_y << LEN_UNIT << EOL;
//cout << "*****************\n";
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
//End of ServoState.cpp********************************************************
//END_CPP_LIB

void setup() {
Serial.begin(2000000);

}

void loop() {
}
