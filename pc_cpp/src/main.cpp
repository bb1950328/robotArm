#include <ctime>
#include <cmath>
#include <iostream>
#include <vector>
#include <sstream>
#include <iterator>
#include "../h/libRobotArm.hpp"
#include "../h/coupling.hpp"
#include "../h/constants.hpp"
#include "../h/ServoState.hpp"
#include "../h/util.hpp"
#include "../h/Point3d.hpp"
#include "../h/ramp3d.hpp"
#include "../h/Point3dLinkNode.hpp"

static const int times = 10000;
static const float SERVO_HORN_RADIUS_DECREMENT_STEP = 2;
using namespace std;

class CouplingTestResult {
public:
    float minJointAngle = NAN;
    float maxJointAngle = NAN;
    float minServoAngle = NAN;
    float maxServoAngle = NAN;

    void printResults(string lineStart);
};

void CouplingTestResult::printResults(string lineStart = "") {
    cout << lineStart << "jointAngle is from " << minJointAngle << ANGLE_UNIT << " to " << maxJointAngle << ANGLE_UNIT;
    cout << " that's a difference of " << maxJointAngle - minJointAngle << ANGLE_UNIT << EOL;
    cout << lineStart << "servoAngle is from " << minServoAngle << ANGLE_UNIT << " to " << maxServoAngle << ANGLE_UNIT
         << EOL;
}

CouplingTestResult *testCoupling(Coupling *coupling);

void testCalc3d();

void coupling_calculator();

void test_ramp();

void interactive_calc3d(string input = "");

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

int main(int argc, char *argv[]) {
    //testCalc3d();
    //testCoupling(new Coupling(10.3, 9.87812, 5.09117, 3.6, 20.4576))->printResults();
    //test_ramp();
    //coupling_calculator();
    if (argc > 1) {
        std::string arg1(argv[1]);
        interactive_calc3d(arg1);
    } else {
        interactive_calc3d();
    }
    return 0;
}

CouplingTestResult *testCoupling(Coupling *coupling) {
    //cout << "servoAngle;jointAngle" << EOL;
    auto *result = new CouplingTestResult();
    for (int i = -900; i <= 900; ++i) {
        float jointAngle = i / 10.0f;
        float servoAngle = coupling->getServoAngle(jointAngle);
        if (!isnan(servoAngle)) {
            //cout << servoAngle << ";" << jointAngle << EOL;
            if (isnan(result->minJointAngle) || jointAngle < result->minJointAngle) {
                result->minJointAngle = jointAngle;
            }
            if (isnan(result->maxJointAngle) || jointAngle > result->maxJointAngle) {
                result->maxJointAngle = jointAngle;
            }
        }
    }
    result->minServoAngle = coupling->getServoAngle(result->minJointAngle);
    result->maxServoAngle = coupling->getServoAngle(result->maxJointAngle);
    return result;
}

void coupling_calculator() {
    float alpha = 40, d = 57 + 20, s = 30;
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
    while (true) {
        if (s <= 0) {
            break;
        }
        float dx, dy, beta, g, c;
        dy = s / tan(util::radians(alpha));
        beta = util::degrees(acos(dy / d));
        dx = sin(util::radians(beta)) * d;
        g = s / sin(util::radians(alpha));
        c = sqrt(pow(dy + s - g, 2) + dx * dx);
        if (isnan(dx) || isnan(dy) || isnan(beta) || isnan(g) || isnan(c)) {
            cout << "Invalid parameters ):" << EOL;
            continue;
        }
        float servoOffset = 90 - beta;
        cout << "Servo horn offset angle (=90-beta) is " << servoOffset << ANGLE_UNIT << EOL;
        cout << "Joint radius (=g) is " << g << LEN_UNIT << EOL;
        cout << "Connector bar length (=c) is " << c << LEN_UNIT << EOL;
        cout << "dx=" << dx << LEN_UNIT << " and dy=" << dy << LEN_UNIT << EOL;
        cout << "Constructor: Coupling(" << d << ", " << c << ", " << g << ", " << s << ", " << servoOffset << ");"
             << EOL;
        auto *testResults = testCoupling(new Coupling(d, c, g, s, servoOffset));
        cout << STAR_LINE_64 << EOL;
        testResults->printResults("Testing results:\t");
        cout << STAR_LINE_64 << EOL;
        float adjustment = (testResults->minJointAngle + testResults->maxJointAngle) / -2;
        cout << "Joint offset adjustment: " << adjustment << ANGLE_UNIT << EOL;
        cout << "Constructor with adjustment: Coupling(" << d << ", " << c << ", " << g << ", " << s << ", "
             << servoOffset
             << ", " << servoOffset + adjustment
             << ");" << EOL;

        auto *secondTestResults = testCoupling(new Coupling(d, c, g, s, servoOffset, servoOffset + adjustment));
        cout << STAR_LINE_64 << EOL;
        secondTestResults->printResults("Second testing results:\t");
        float rangeDifference =
                (secondTestResults->maxJointAngle - secondTestResults->minJointAngle) - (alpha * 2);
        if (rangeDifference < 5) {
            break;
        } else {
            s -= SERVO_HORN_RADIUS_DECREMENT_STEP;
            cout << EOL << STAR_LINE_64 << STAR_LINE_64 << EOL;
            cout << "The joint range is much bigger than desired ( " << rangeDifference << ANGLE_UNIT
                 << " too big), now trying with servoRadius=";
            cout << s << LEN_UNIT << EOL;
        }
    }
}

void test_ramp() {
    auto *start = new Point3d(10, 2, 3);
    auto *end = new Point3d(5, 6, 4);
    cout << "Calculating Ramp from " << start->toString() << " to " << end->toString() << EOL;
    auto *ramp = new Ramp3d();
    ramp->calculate_linear(start, end, 1);
    for (Point3dLinkNode *i = ramp->getStartNode(); i != nullptr; i = i->next) {
        cout << " -> " << i->toString() << EOL;
    }
}

vector<string> split(const string &s, char delim) {
    vector<string> result;
    stringstream ss(s);
    string item;

    while (getline(ss, item, delim)) {
        result.push_back(item);
    }

    return result;
}

template<typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> vec) {
    os << "{ ";
    std::copy(vec.begin(), vec.end(), std::ostream_iterator<T>(os, " "));
    os << "}";
    return os;
}

void interactive_calc3d(string input) {
    auto *arm = new RobotArm();
    float x, y, z, omega;
    if (input.length() == 0) {
        getline(cin, input);
    }
    //vector<string> splitted = split(input, ';');
    std::sscanf(input.c_str(), "%fand%fand%fand%f", &x, &y, &z, &omega);

    /*cout << splitted << EOL;
    x = stof(splitted[0]);
    y = stof(splitted[1]);
    z = stof(splitted[2]);
    omega = stof(splitted[3]);*/
    auto *to = new Point3d(x, y, z);
    arm->goTo(to, omega);
    //arm->getState()->print();
    delete to;
    RobotArm::print_config();
}

