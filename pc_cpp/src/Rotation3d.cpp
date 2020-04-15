//
// Created by Bader on 15.04.2020.
//

#include <cmath>
#include "../h/Rotation3d.hpp"

Rotation3d Rotation3d::fromAcceleration(float accX, float accY, float accZ) {
    //todo check if implementation is correct
    // from https://forum.arduino.cc/index.php?topic=112031.0 Post #5
    Rotation3d result{};

    result.rotX = atan2(-accY, -accZ) * CONST_57 + 180;
    result.rotY = atan2(-accX, -accZ) * CONST_57 + 180;
    result.rotZ = atan2(-accY, -accX) * CONST_57 + 180;
    return result;
}