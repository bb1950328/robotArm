//
// Created by bab21 on 25.03.20.
//

#ifndef ROBOTARM_SERVOSTATE_HPP
#define ROBOTARM_SERVOSTATE_HPP


class ServoState {
public:
    float alpha;//arm #1
    float beta;//arm #2
    float gamma;//arm #3
    float delta;//turntable
    float epsilon;//gripper rotation
    float zeta;//gripper fingers

    float u;//alpha joint to gamma joint angle
    float p2_x; // p2 is gamma joint
    float p2_y;

    ServoState();

    bool operator==(const ServoState &rhs) const;

    bool operator!=(const ServoState &rhs) const;

    void print();

    bool isValid();
};


#endif //ROBOTARM_SERVOSTATE_HPP
