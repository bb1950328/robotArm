//
// Created by Bader on 15.04.2020.
//

#ifndef ROBOTARM_ROTATION3D_HPP
#define ROBOTARM_ROTATION3D_HPP

class Rotation3d {
public:
    float rotX, rotY, rotZ;

    static Rotation3d fromAcceleration(float accX, float accY, float accZ);
};

#endif //ROBOTARM_ROTATION3D_HPP
