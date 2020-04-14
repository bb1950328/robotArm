//
// Created by Bader on 27.03.2020.
//

#ifndef ROBOTARM_COUPLING_HPP
#define ROBOTARM_COUPLING_HPP

class Coupling {
public:
    Coupling(float axleDistance,
             float couplerLength,
             float jointRadius,
             float servoHornRadius,
             float servoOffsetAngle,
             float jointOffsetAngle);

    Coupling(float axleDistance,
             float couplerLength,
             float jointRadius,
             float servoHornRadius,
             float servoOffsetAngle);

    /**
     * @param jointAngle between -x and +x
     * @return servoAngle between -90° and +90°
     */
    float
    getServoAngle(float jointAngle, bool optimizedMethod = false);// todo find out which method is faster on arduino

    /**
     * @warning this method is not tested!!!
     */
    float getJointAngle(float servoAngle, bool optimizedMethod = false);

    float getAxleDistance() const;

    float getCouplerLength() const;

    float getServoHornRadius() const;

    float getJointRadius() const;

    float getServoOffsetAngle() const;

    float getJointOffsetAngle() const;

private:
    float d;
    float c;
    float s;
    float g;
    float servoOffsetAngle;
    float jointOffsetAngle;
};

#endif //ROBOTARM_COUPLING_HPP
