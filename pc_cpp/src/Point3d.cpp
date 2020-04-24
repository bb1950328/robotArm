#include <cmath>
#include <sstream>
#include "../h/Point3d.hpp"


Point3d::Point3d() {
    x = y = z = 0.0f;
}

Point3d::Point3d(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

float Point3d::distance_between(Point3d *p1, Point3d *p2) {
    float diffX = p1->x - p2->x;
    float diffY = p1->y - p2->y;
    float diffZ = p1->z - p2->z;
    return std::sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

float Point3d::distance_to(Point3d *other) {
    return distance_between(this, other);
}

/**
 *     distance
 *   |<---------->|
 * start--------return----------target
 */
Point3d *Point3d::in_direction(Point3d *start, Point3d *target, float distance) {
    float factor = distance / distance_between(start, target);
    float diffX = target->x - start->x;
    float diffY = target->y - start->y;
    float diffZ = target->z - start->z;
    return new Point3d(
            start->x + diffX * factor,
            start->y + diffY * factor,
            start->z + diffZ * factor);
}

#ifdef ARDUINO
#ifdef USE_LCD
void Point3d::toLCD() {
    //X12.3Y45.6Z78.9
    lcd.print("X");
    lcd.print(x, 1);
    lcd.print("Y");
    lcd.print(y, 1);
    lcd.print("Z");
    lcd.print(z, 1);
}
#endif
#else

std::string Point3d::toString() {

    std::stringstream result;
    result << "X=" << x << ", Y=" << y << ", Z=" << z;
    return result.str();
}

#endif

