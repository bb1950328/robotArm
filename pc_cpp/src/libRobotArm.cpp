

#include <cmath>
#include <iostream>
#include "../h/ServoState.hpp"
#include "../h/util.hpp"
#include "../h/libRobotArm.hpp"
#include "../h/constants.hpp"

using namespace std;


ServoState RobotArm::internal_calc2d(float r, float z, float omega) {

    ServoState state;

    float x_gamma = cos(util::radians(omega)) * L3;
    float y_gamma = sin(util::radians(omega)) * L3;

    state.p2_x = r - x_gamma;
    state.p2_y = z + y_gamma;

    float c = sqrt(state.p2_x * state.p2_x + state.p2_y * state.p2_y);
    state.u = util::degrees(atan(state.p2_y / state.p2_x));

    state.alpha = state.u + util::degrees(acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)));
    state.beta = 180 - util::degrees(acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)));
    state.gamma = omega - state.alpha - state.beta;

    return state;
}


ServoState RobotArm::calc2d(float r, float z, float omega) {
    ServoState state = internal_calc2d(r, z, omega);
    if (state.isValid()) {
        return state;
    }

    // r and z are too far away
    float new_p2_x = cos(util::radians(state.u)) * U_MAX;
    float new_p2_y = sin(util::radians(state.u)) * U_MAX;

    r -= state.p2_x - new_p2_x;
    z -= state.p2_y - new_p2_y;

    return internal_calc2d(r, z, omega);
}

ServoState RobotArm::calc3d(float x, float y, float z, float omega) {
    float r = sqrt(x * x + y * y);
    ServoState state = calc2d(r, z, omega);
    state.delta = util::degrees(atan(y / x));
    return state;
}

void RobotArm::print_config() {
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

const float RobotArm::U_MAX = sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * cos(util::radians(180 - BETA_MIN)));// cosine law

ServoState *RobotArm::getState() const {
    return state;
}

void RobotArm::goTo(Point3d *to, float omega) {
    this->state->updateCalculated(new ServoState(RobotArm::calc3d(to->x, to->y, to->z, omega)));
}
