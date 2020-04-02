//
// Created by Bader on 27.03.2020.
//

#ifndef ROBOTARM_COUPLING_HPP
#define ROBOTARM_COUPLING_HPP

class Coupling {
public:
    float l;
    float la, lb;

    Coupling(float couplerLength, float jointRadius, float servoHornRadius);

    float getServoAngle(float jointAngle);
};

#endif //ROBOTARM_COUPLING_HPP
