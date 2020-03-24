#define PI 3.14159265358979323846264338327f

#include "math.h"
#include <iostream>

#define LEN_UNIT "cm"
#define ANGLE_UNIT "°"

#define EOL "\n"
using namespace std;

const float L1 = 11.2;
const float L2 = 5.6;
const float L3 = 3.2;

const float ALPHA_MIN = 0.0f;
const float ALPHA_MAX = 90.0f;
const float BETA_MIN = 50.0f;
const float BETA_MAX = 130.0f;
const float GAMMA_MIN = -60.0f;
const float GAMMA_MAX = 60.0f;
const float DELTA_MIN = -90.0f;
const float DELTA_MAX = 90.0f;
const float EPSILON_MIN = 0.0f;
const float EPSILON_MAX = 180.0f;
const float ZETA_MIN = 0.0f;
const float ZETA_MAX = 180.0f;

const float OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1;
const float OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX;

class ServoState;

ServoState calc3d(float x, float y, float z, float omega);

ServoState calc2d(float r, float z, float omega);

ServoState internal_calc2d(float r, float z, float omega);

float degrees(float rad);

float radians(float deg);

const float U_MAX = sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * cos(radians(180 - BETA_MIN))); // cosine law

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
    ServoState() {
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

    bool operator==(const ServoState &rhs) const {
        return alpha == rhs.alpha && beta == rhs.beta && gamma == rhs.gamma && delta == rhs.delta && epsilon == rhs.epsilon && zeta == rhs.zeta && u == rhs.u && p2_x == rhs.p2_x && p2_y == rhs.p2_y;
    }

    bool operator!=(const ServoState &rhs) const {
        return !(rhs == *this);
    }

    void print() {
        cout << "***Servo state***\n";
        cout << "* alpha=" << alpha << ANGLE_UNIT << EOL;
        cout << "* beta=" << beta << ANGLE_UNIT << EOL;
        cout << "* gamma=" << gamma << ANGLE_UNIT << EOL;
        cout << "* delta=" << delta << ANGLE_UNIT << EOL;
        cout << "* epsilon=" << epsilon << ANGLE_UNIT << EOL;
        cout << "* zeta=" << zeta << ANGLE_UNIT << EOL;
        cout << "> u=" << u << ANGLE_UNIT << EOL;
        cout << "> p2_x=" << p2_x << LEN_UNIT << EOL;
        cout << "> p2_y=" << p2_y << LEN_UNIT << EOL;
        cout << "*****************\n";
    }

    bool isValid() {
        return !(
                isnan(alpha) || ALPHA_MIN > alpha || ALPHA_MAX < alpha ||
                isnan(beta) || BETA_MIN > beta || BETA_MAX < beta ||
                isnan(gamma) || GAMMA_MIN > gamma || GAMMA_MAX < gamma ||
                isnan(delta) || DELTA_MIN > delta || DELTA_MAX < delta ||
                isnan(epsilon) || EPSILON_MIN > epsilon || EPSILON_MAX < epsilon ||
                isnan(zeta) || ZETA_MIN > zeta || ZETA_MAX < zeta
        );
    }
};

float degrees(float rad) {
    return rad * 180.0f / PI;
}

float radians(float deg) {
    return deg * PI / 180.0f;
}

ServoState calc2d(float r, float z, float omega) {
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


ServoState internal_calc2d(float r, float z, float omega) {

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

ServoState calc3d(float x, float y, float z, float omega) {
    float r = sqrt(x*x+y*y);
    ServoState state = calc2d(r, z, omega);
    state.delta = degrees(atan(y/x));
    return state;
}

void print_config() {
    cout << "***Robot Arm Config***\n";
    cout << "* L1=" << L1 << LEN_UNIT << EOL;
    cout << "* L2=" << L2 << LEN_UNIT << EOL;
    cout << "* L3=" << L3 << LEN_UNIT << EOL;
    cout << "* " << ALPHA_MIN << ANGLE_UNIT << " ... alpha ... " << ALPHA_MAX << ANGLE_UNIT << EOL;
    cout << "* " << BETA_MIN << ANGLE_UNIT << " ... beta ... " << BETA_MAX << ANGLE_UNIT << EOL;
    cout << "* " << GAMMA_MIN << ANGLE_UNIT << " ... gamma ... " << GAMMA_MAX << ANGLE_UNIT << EOL;
    cout << "* " << DELTA_MIN << ANGLE_UNIT << " ... delta ... " << DELTA_MAX << ANGLE_UNIT << EOL;
    cout << "* " << EPSILON_MIN << ANGLE_UNIT << " ... epsilon ... " << EPSILON_MAX << ANGLE_UNIT << EOL;
    cout << "* " << ZETA_MIN << ANGLE_UNIT << " ... zeta ... " << ZETA_MAX << ANGLE_UNIT << EOL;
    cout << "**********************\n";
}