//
// Created by bab21 on 25.03.20.
//

#ifndef ROBOTARM_CONSTANTS_HPP
#define ROBOTARM_CONSTANTS_HPP

#define LEN_UNIT "cm"
#define ANGLE_UNIT "°"

#define EOL "\n"

const float L1 = 11.2;
const float L2 = 5.6;
const float L3 = 3.2;

const float ALPHA_MIN = 0.0f;
const float ALPHA_MAX = 90.0f;
const float BETA_MIN = 50.0f;
const float BETA_MAX = 130.0f;
const float GAMMA_MIN = -60.0f;
const float GAMMA_MAX = 60.0f;
const float DELTA_MIN = -90.0f;
const float DELTA_MAX = 90.0f;
const float EPSILON_MIN = 0.0f;
const float EPSILON_MAX = 180.0f;
const float ZETA_MIN = 0.0f;
const float ZETA_MAX = 180.0f;

const float OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1;
const float OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX;

#endif //ROBOTARM_CONSTANTS_HPP
