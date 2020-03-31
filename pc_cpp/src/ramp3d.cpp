#include <cmath>
#include "../h/Point3dLinkNode.hpp"

class Ramp3d {
public:
    double acceleration = 60; // unit/second^2
    double max_speed = 100; // unit/second
    double steps_per_second = 50;

    Ramp3d() {
    }

    void calculate(Point3d *start, Point3d *stop) {
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
        while (travelled_distance*2 + travel_per_step < total_distance) {
            Point3dLinkNode *new_a = new Point3dLinkNode(/*TODO*/);
        }
    }

private:
    Point3dLinkNode *startNode;
    Point3dLinkNode *stopNode;

    Point3dLinkNode* getStartRamp() {
        //todo
    }
};



