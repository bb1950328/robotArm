#include <ctime>
#include <iostream>
#include "../h/libRobotArm.hpp"
#include "../h/coupling.hpp"
#include "../h/constants.hpp"
#include "../h/ServoState.hpp"

static const int times = 10000;
using namespace std;

void testCoupling();

void testCalc3d();

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
    testCalc3d();
    testCoupling();
    return 0;
}

void testCoupling() {
    Coupling cp = Coupling(96, 64, 48);
    clock_t start = clock();
    float result;
    for (int i = 0; i < times; ++i) {
        result = cp.getServoAngle(35);
    }
    clock_t end = clock();
    cout << "servo angle: " << result << ANGLE_UNIT << EOL;
    cout << "cp.getServoAngle(35) used " << (float) (end - start) * 1000000 / (float) times / CLOCKS_PER_SEC << "us"
         << EOL;
}
