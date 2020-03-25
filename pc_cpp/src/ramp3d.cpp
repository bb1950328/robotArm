#include <cmath>
#include "../h/Point3dLinkNode.hpp"

class Ramp3d {
public:
    float max_acceleration = 5; // unit/second^2
    float max_speed = 100; // unit/second
    float steps_per_second = 50;

    Ramp3d() {

    }

private:
    Point3dLinkNode *startNode;
};



