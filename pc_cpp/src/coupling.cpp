#include <cmath>
#include "../h/util.hpp"
#include "../h/coupling.hpp"

Coupling::Coupling(float couplerLength, float jointRadius, float servoHornRadius) {
    this->l = couplerLength;
    this->la = jointRadius;
    this->lb = servoHornRadius;
}

float Coupling::getServoAngle(float jointAngle) {
    float dx = std::sin(util::radians(jointAngle)) * la;
    float dy = std::cos(util::radians(jointAngle)) * la;

    float l_dx = l - dx;
    float n = std::sqrt(dy * dy + l_dx * l_dx);

    float gamma = util::degrees(std::acos((n * n + lb * lb - l * l) / (2 * n * lb)));
    float delta = util::degrees(std::atan(dy / l_dx));
    return delta + gamma - 90;
}