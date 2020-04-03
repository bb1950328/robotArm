#include "../h/Point3dLinkNode.hpp"
#include "../h/ramp3d.hpp"
#include <cmath>

Ramp3d::Ramp3d() {
}

void Ramp3d::calculate_nonlinear(Point3d *start, Point3d *stop) {
    /*
     * start-------node_a   node_b-------stop
     */
    this->startNode = new Point3dLinkNode(start);
    this->stopNode = new Point3dLinkNode(stop);
    int i_step = 0;
    double total_distance = Point3d::distance_between(start, stop);
    double travelled_distance = 0;
    double travel_per_step = acceleration / steps_per_second;
    Point3dLinkNode *node_a = startNode;
    Point3dLinkNode *node_b = stopNode;
    while (travelled_distance * 2 + travel_per_step < total_distance) {
        Point3dLinkNode *new_a = new Point3dLinkNode(/*TODO*/);
    }
}

void Ramp3d::calculate_linear(Point3d *start, Point3d *stop, float desiredStepLength) {
    float total_distance = Point3d::distance_between(start, stop);
    this->startNode = new Point3dLinkNode(start);
    Point3dLinkNode *iNode = this->startNode;
    int numSteps = std::ceil(total_distance / desiredStepLength);
    float actualStepLength = total_distance / numSteps;
    float iDistance = 0;
    for (int i = 0; i < numSteps; ++i) {
        iDistance += actualStepLength;
        auto *newNode = new Point3dLinkNode(Point3d::in_direction(start, stop, iDistance));
        newNode->last = iNode;
        iNode->next = newNode;
        iNode = newNode;
    }
}

Point3dLinkNode *Ramp3d::getStartRamp() {
    //todo
    return nullptr;
}



