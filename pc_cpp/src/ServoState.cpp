//
// Created by bab21 on 25.03.20.
//

#include "iostream"
#include "../h/constants.hpp"
#include "math.h"

using namespace std;

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