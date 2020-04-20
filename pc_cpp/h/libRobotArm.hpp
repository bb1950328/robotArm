//
// Created by Bader on 02.04.2020.
//

#ifndef ROBOTARM_LIBROBOTARM_HPP
#define ROBOTARM_LIBROBOTARM_HPP

#include "constants.hpp"
#include "ServoState.hpp"
#include "Point3d.hpp"

class RobotArm {
public:
    static ServoState internal_calc2d(float r, float z, float omega);

    static ServoState calc2d(float r, float z, float omega);

    static ServoState calc3d(float x, float y, float z, float omega);

    static void print_config();

    void goTo(Point3d *to, float omega);

    ServoState *getState() const;

private:
    ServoState *state;
    static const float U_MAX;
};

#endif //ROBOTARM_LIBROBOTARM_HPP
