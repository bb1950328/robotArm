#include <cmath>
#include "../h/util.hpp"
#include "../h/coupling.hpp"


Coupling::Coupling(
        float axleDistance,
        float couplerLength,
        float jointRadius,
        float servoHornRadius,
        float offsetAngle) {
    d = axleDistance;
    c = couplerLength;
    g = jointRadius;
    s = servoHornRadius;
    this->offsetAngle = offsetAngle;
}

float Coupling::getServoAngle(float jointAngle, bool optimizedMethod) {
    float delta = 90 - offsetAngle + jointAngle;
    float a = std::sqrt(d * d + g * g - 2 * d * g * std::cos(util::radians(delta)));
    float fr1 = (d * d + a * a - g * g) / (2 * d * a);
    float fr2 = (a * a + s * s - c * c) / (2 * a * s);
    float lambdaRad;
    if (optimizedMethod) {
        lambdaRad = std::acos(fr1 * fr2 - std::sqrt(1 - fr1 * fr1) * std::sqrt(1 - fr2 * fr2));
    } else {
        lambdaRad = std::acos(fr1) + std::acos(fr2);
    }
    return offsetAngle - util::degrees(lambdaRad) + 90;
}

float Coupling::getAxleDistance() const {
    return d;
}

float Coupling::getCouplerLength() const {
    return c;
}

float Coupling::getServoHornRadius() const {
    return s;
}

float Coupling::getJointRadius() const {
    return g;
}

float Coupling::getOffsetAngle() const {
    return offsetAngle;
}
