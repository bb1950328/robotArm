#include <cmath>
#include "../h/utils.hpp"
#include "../h/coupling.hpp"

Coupling::Coupling(float couplerLength, float jointRadius, float servoHornRadius) {
    this->l = couplerLength;
    this->la = jointRadius;
    this->lb = servoHornRadius;
}

float Coupling::getServoAngle(float jointAngle) {
    float dx = std::sin(radians(jointAngle)) * la;
    float dy = std::cos(radians(jointAngle)) * la;

    float l_dx = l - dx;
    float n = std::sqrt(dy * dy + l_dx * l_dx);

    float gamma = degrees(std::acos((n * n + lb * lb - l * l) / (2 * n * lb)));
    float delta = degrees(std::atan(dy / l_dx));
    return delta + gamma - 90;
}