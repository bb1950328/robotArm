#include "../h/Point3d.hpp"

class Point3d {
public:
    float x, y, z;
    Point3d() {
        x = y = z = 0.0f;
    }
    Point3d(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    static float distance_between(Point3d* p1, Point3d* p2) {
        float diffX = p1->x - p2->x;
        float diffY = p1->y - p2->y;
        float diffZ = p1->z - p2->z;
        return sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    }

    float distance_to(Point3d* other) {
        return distance_between(this, other);
    }
};