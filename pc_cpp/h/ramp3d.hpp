//
// Created by Bader on 03.04.2020.
//

#ifndef ROBOTARM_RAMP3D_HPP
#define ROBOTARM_RAMP3D_HPP

class Ramp3d {
public:
    double acceleration = 60; // unit/second^2
    double max_speed = 100; // unit/second
    double steps_per_second = 50;

    Ramp3d();


private:
    Point3dLinkNode *startNode;
    Point3dLinkNode *stopNode;

    Point3dLinkNode *getStartRamp();

    void calculate_nonlinear(Point3d *start, Point3d *stop);

    void calculate_linear(Point3d *start, Point3d *stop, float desiredStepLength);
};

#endif //ROBOTARM_RAMP3D_HPP
