#include "../h/Point3d.hpp"

class Point3d {
public:
    double x, y, z;
    Point3d() {
        x = y = z = 0.0f;
    }
    Point3d(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    static double distance_between(Point3d* p1, Point3d* p2) {
        double diffX = p1->x - p2->x;
        double diffY = p1->y - p2->y;
        double diffZ = p1->z - p2->z;
        return sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    }

    double distance_to(Point3d* other) {
        return distance_between(this, other);
    }

    /**
     *     distance
     *   |<---------->|
     * start--------return----------target
     */
    static Point3d* in_direction(Point3d* start, Point3d* target, double distance) {
        double factor = distance / distance_between(start, target);
        double diffX = p1->x - p2->x;
        double diffY = p1->y - p2->y;
        double diffZ = p1->z - p2->z;
        return Point3d(
                start.x + diffX*factor,
                start.y + diffY*factor,
                start.z + diffZ*factor,
                )
    }
};