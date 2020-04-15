//
// Created by Bader on 15.04.2020.
//

#ifndef ROBOTARM_ROTATION3D_HPP
#define ROBOTARM_ROTATION3D_HPP

static const float CONST_57 = 57.2957795;

class Rotation3d {
public:
    float rotX, rotY, rotZ;

    /**
     * @param accX, accY, accZ in G
     */
    static Rotation3d fromAcceleration(float accX, float accY, float accZ);
};

#endif //ROBOTARM_ROTATION3D_HPP
