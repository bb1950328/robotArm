#ifndef ROBOTARM_POINT3D_H
#define ROBOTARM_POINT3D_H


#include <string>

class Point3d {
public:
    float x, y, z;

    Point3d();

    Point3d(float x, float y, float z);

    static float distance_between(Point3d *p1, Point3d *p2);

    float distance_to(Point3d *other);

    /**
     *     distance
     *   |<---------->|
     * start--------return----------target
     */
    static Point3d *in_direction(Point3d *start, Point3d *target, float distance);

#ifdef ARDUINO
    void toLCD();
#else

    std::string toString();

#endif
};

#endif //ROBOTARM_POINT3D_H
