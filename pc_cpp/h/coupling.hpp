//
// Created by Bader on 27.03.2020.
//

#ifndef ROBOTARM_COUPLING_HPP
#define ROBOTARM_COUPLING_HPP

class Coupling {
public:
    Coupling(float axleDistance, float couplerLength, float jointRadius, float servoHornRadius, float offsetAngle);

    /**
     * @param jointAngle between -x and +x
     * @return servoAngle between -90° and +90°
     */
    float getServoAngle(float jointAngle, bool optimizedMethod = false);// todo find out which one is faster on arduino

    float getAxleDistance() const;

    float getCouplerLength() const;

    float getServoHornRadius() const;

    float getJointRadius() const;

    float getOffsetAngle() const;

private:
    float d;
    float c;
    float s;
    float g;
    float offsetAngle;
};

#endif //ROBOTARM_COUPLING_HPP
