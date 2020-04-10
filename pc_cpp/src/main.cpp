#include <ctime>
#include <cmath>
#include <iostream>
#include "../h/libRobotArm.hpp"
#include "../h/coupling.hpp"
#include "../h/constants.hpp"
#include "../h/ServoState.hpp"
#include "../h/util.hpp"

static const int times = 10000;
using namespace std;

void testCoupling();

void testCalc3d();

void coupling_calculator();

void testCalc3d() {
    RobotArm::print_config();
    ServoState state;
    clock_t start = clock();
    for (int i = 0; i < times; ++i) {
        state = RobotArm::calc3d(8.2f, 8.2f, 3.9f, 60.0f);
    }
    clock_t end = clock();
    float seconds = (float) (end - start) * 1000000 / (float) times / CLOCKS_PER_SEC;
    cout << "calc3d() function used %.12f us\n" << seconds;
    state.print();
}

int main() {
    //testCalc3d();
    testCoupling();
    //coupling_calculator();
    return 0;
}

void testCoupling() {
    Coupling cp = Coupling(10.3, 9.87812, 5.09117, 3.6, 20.4576);
    cout << "jointAngle;servoAngle" << EOL;
    for (int servoAngle = -200; servoAngle <= 200; ++servoAngle) {
        float jointAngle = cp.getJointAngle(servoAngle);
        if (!isnan(jointAngle)) {
            cout << jointAngle << ";" << servoAngle << EOL;
        }
    }
}

void coupling_calculator() {
    float alpha = 45, d = 10.3, s = 3.6;
    char enter_own;
    cout << "Would you like to enter your own values? (y/n) ";
    cin >> enter_own;
    if (enter_own == 'y') {
        cout << "Please enter the distance in " << LEN_UNIT << ": ";
        cin >> d;
        cout << "Please enter the servo horn radius in " << LEN_UNIT << ": ";
        cin >> s;
        cout << "Please enter the desired joint range (-x .... +x) in " << ANGLE_UNIT << ": ";
        cin >> alpha;
    }

    if (alpha >= 90 || alpha <= 0) {
        cout << "the desired joint range must be between 0 and 90" << ANGLE_UNIT << " (exclusively)" << EOL;
        return;
    }
    float dx, dy, beta, g, c;
    dy = s / tan(util::radians(alpha));
    beta = util::degrees(acos(dy / d));
    dx = sin(util::radians(beta)) * d;
    g = s / sin(util::radians(alpha));
    c = sqrt(pow(dy + s - g, 2) + dx * dx);

    if (isnan(dx) || isnan(dy) || isnan(beta) || isnan(g) || isnan(c)) {
        cout << "Invalid parameters ):" << EOL;
    } else {
        float offset = 90 - beta;
        cout << "Servo horn and joint lever offset angle (=90-beta) is " << offset << ANGLE_UNIT << EOL;
        cout << "Joint radius (=g) is " << g << LEN_UNIT << EOL;
        cout << "Connector bar length (=c) is " << c << LEN_UNIT << EOL;
        cout << "dx=" << dx << LEN_UNIT << " and dy=" << dy << LEN_UNIT << EOL;
        cout << "Constructor: Coupling(" << d << ", " << c << ", " << g << ", " << s << ", " << offset << ");" << EOL;
    }
}