//
// Created by Bader on 27.03.2020.
//

#ifndef ROBOTARM_UTIL_HPP
#define ROBOTARM_UTIL_HPP
#define PI 3.14159265358979323846264338327f
#include <chrono>
namespace util {

    float degrees(float rad);

    float radians(float deg);

    inline float degrees(float rad) {
        return rad * 180.0f / PI;
    }

    inline float radians(float deg) {
        return deg * PI / 180.0f;
    }

    inline long millis() {
        auto duration = std::chrono::high_resolution_clock::now().time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
}

#endif //ROBOTARM_UTIL_HPP
