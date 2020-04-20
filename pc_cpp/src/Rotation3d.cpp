//
// Created by Bader on 15.04.2020.
//

#include <cmath>
#include "../h/Rotation3d.hpp"
#include "../h/util.hpp"

Rotation3d Rotation3d::fromAcceleration(float accX, float accY, float accZ) {
    //todo check if implementation is correct
    // from https://forum.arduino.cc/index.php?topic=112031.0 Post #5
    Rotation3d result{};

    result.rotX = util::degrees(atan2(-accY, -accZ)) + 180;
    result.rotY = util::degrees(atan2(-accX, -accZ)) + 180;
    result.rotZ = util::degrees(atan2(-accY, -accX)) + 180;
    return result;
}