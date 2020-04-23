
#include <cmath>
#include <iostream>
#include "../h/ServoState.hpp"
#include "../h/util.hpp"
#include "../h/libRobotArm.hpp"
#include "../h/constants.hpp"
#include "../h/coupling.hpp"

using namespace std;


RobotArm::RobotArm() {
    /**--------------given------------- | --------------------------------calculated-----------------------------------|
     *  horn radius | distance | range  | servo horn offs | joint offs adj  | joint radius | connector | dx     | dy   |
     * a 17mm       | 94mm     | +/-45° | 10.41°          | -1.8°           | 24.04mm      | 92.98mm   | 92.5mm | 17mm |
     * b 28mm       | 77mm     | +/-40° | 25.6813°        | -3.85°          | 43.5603mm    | 71.6426mm | 69.4mm | 33mm |
     * c 30mm       | 125mm    | +/-50° | 11.6179°        | -2.4°           | 39.1622mm    | 123.481mm | 122mm  | 25mm |
     */
    couplingA = new Coupling(94, 92.9848, 24.0416, 17, 10.4193, 8.61934); // NOLINT(cert-err58-cpp)
    couplingB = new Coupling(77, 71.6426, 43.5603, 28, 25.6813, 21.8313); // NOLINT(cert-err58-cpp)
    couplingC = new Coupling(125, 123.481, 39.1622, 30, 11.6179, 9.2179); // NOLINT(cert-err58-cpp)

    state = new ServoState();
}

ServoState RobotArm::internal_calc2d(float r, float z, float omega) {
    long start = util::millis();
    ServoState state;

    float x_gamma = cos(util::radians(omega)) * L3;
    float y_gamma = sin(util::radians(omega)) * L3;

    state.p2_x = r - x_gamma;
    state.p2_y = z + y_gamma;

    float c = sqrt(state.p2_x * state.p2_x + state.p2_y * state.p2_y);
    state.u = util::degrees(atan(state.p2_y / state.p2_x));

    state.alpha = state.u + util::degrees(acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)));
    state.beta = 180 - util::degrees(acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)));
    state.gamma = omega + (state.alpha - state.beta);
    long end = util::millis();
    state.print();
    cout << "r: " << r << EOL;
    cout << "z: " << z << EOL;
    cout << "x_gamma: " << x_gamma << EOL;
    cout << "y_gamma: " << y_gamma << EOL;
    cout << "u: " << state.u << EOL;
    cout << "time used in ms: " << end << EOL;
    return state;
}


ServoState RobotArm::calc2d(float r, float z, float omega) {
    ServoState state = internal_calc2d(r, z, omega);
#ifdef AUTOCORRECT_TOO_FAR_COORDS
    if (!state.isValid()) {
        // r and z are too far away
        float new_p2_x = cos(util::radians(state.u)) * U_MAX;
        float new_p2_y = sin(util::radians(state.u)) * U_MAX;

        r -= state.p2_x - new_p2_x;
        z -= state.p2_y - new_p2_y;

        state = internal_calc2d(r, z, omega);
    }
#endif

    return state;
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
