//
// Created by bab21 on 25.03.20.
//

#ifndef ROBOTARM_CONSTANTS_HPP
#define ROBOTARM_CONSTANTS_HPP

#define LEN_UNIT "mm"
#define ANGLE_UNIT "deg"//"°"
#define STAR_LINE_64 "****************************************************************"

#define EOL "\n"

constexpr float L1 = 300;
constexpr float L2 = 200;
constexpr float L3 = 175;

constexpr float ALPHA_MIN = 0.0f;
constexpr float ALPHA_MAX = 90.0f;
constexpr float BETA_MIN = 50.0f;
constexpr float BETA_MAX = 130.0f;
constexpr float GAMMA_MIN = -60.0f;
constexpr float GAMMA_MAX = 60.0f;
constexpr float DELTA_MIN = -90.0f;
constexpr float DELTA_MAX = 90.0f;
constexpr float EPSILON_MIN = 0.0f;
constexpr float EPSILON_MAX = 180.0f;
constexpr float ZETA_MIN = 0.0f;
constexpr float ZETA_MAX = 180.0f;

constexpr float OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1;
constexpr float OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX;

#endif //ROBOTARM_CONSTANTS_HPP
