# coding=utf-8

import math
import time

PI = 3.14159265358979323846264338327

LEN_UNIT = "cm"
ANGLE_UNIT = "°"

L1 = 11.2
L2 = 5.6
L3 = 3.2
ALPHA_MIN = 0.0
ALPHA_MAX = 90.0
BETA_MIN = 50.0
BETA_MAX = 130.0
GAMMA_MIN = -60.0
GAMMA_MAX = 60.0
DELTA_MIN = -90.0
DELTA_MAX = 90.0
EPSILON_MIN = 0.0
EPSILON_MAX = 180.0
ZETA_MIN = 0.0
ZETA_MAX = 180.0

OMEGA_LOWER = (ALPHA_MAX - BETA_MIN - GAMMA_MIN) * -1
OMEGA_UPPER = ALPHA_MIN - BETA_MAX - GAMMA_MAX


def isnan(fl):
    return False


def degrees(rad):
    return rad * 180.0 / PI


def radians(deg):
    return deg * PI / 180.0


U_MAX = math.sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * math.cos(radians(180 - BETA_MIN)))  # cosine law


class ServoState(object):
    alpha: float = 0.0  # arm #1
    beta: float = 0.0  # arm #2
    gamma: float = 0.0  # arm #3
    delta: float = 0.0  # turntable
    epsilon: float = 0.0  # gripper rotation
    zeta: float = 0.0  # gripper fingers
    u: float = 0.0  # alpha joint to gamma joint angle
    p2_x: float = 0.0  # p2 is gamma joint
    p2_y: float = 0.0

    def output(self):
        print("***Servo state***\n")
        print("* alpha=" + str(self.alpha) + ANGLE_UNIT)
        print("* beta=" + str(self.beta) + ANGLE_UNIT)
        print("* gamma=" + str(self.gamma) + ANGLE_UNIT)
        print("* delta=" + str(self.delta) + ANGLE_UNIT)
        print("* epsilon=" + str(self.epsilon) + ANGLE_UNIT)
        print("* zeta=" + str(self.zeta) + ANGLE_UNIT)
        print("> u=" + str(self.u) + ANGLE_UNIT)
        print("> p2_x=" + str(self.p2_x) + LEN_UNIT)
        print("> p2_y=" + str(self.p2_y) + LEN_UNIT)
        print("*****************\n")

    def is_valid(self):
        return not (
                isnan(self.alpha) or ALPHA_MIN > self.alpha or ALPHA_MAX < self.alpha or
                isnan(self.beta) or BETA_MIN > self.beta or BETA_MAX < self.beta or
                isnan(self.gamma) or GAMMA_MIN > self.gamma or GAMMA_MAX < self.gamma or
                isnan(self.delta) or DELTA_MIN > self.delta or DELTA_MAX < self.delta or
                isnan(self.epsilon) or EPSILON_MIN > self.epsilon or EPSILON_MAX < self.epsilon or
                isnan(self.zeta) or ZETA_MIN > self.zeta or ZETA_MAX < self.zeta
        )


def calc2d(r, z, omega):
    state = internal_calc2d(r, z, omega)
    if state.is_valid():
        return state

    # r and z are too far away
    new_p2_x = math.cos(radians(state.u)) * U_MAX
    new_p2_y = math.sin(radians(state.u)) * U_MAX

    r -= state.p2_x - new_p2_x
    z -= state.p2_y - new_p2_y

    return internal_calc2d(r, z, omega)


def internal_calc2d(r, z, omega):
    state = ServoState()
    x_gamma = math.cos(radians(omega)) * L3
    y_gamma = math.sin(radians(omega)) * L3

    state.p2_x = r - x_gamma
    state.p2_y = z + y_gamma

    c = math.sqrt(state.p2_x * state.p2_x + state.p2_y * state.p2_y)
    state.u = degrees(math.atan(state.p2_y / state.p2_x))

    state.alpha = state.u + degrees(math.acos((c * c + L1 * L1 - L2 * L2) / (2 * c * L1)))
    state.beta = 180 - degrees(math.acos((L1 * L1 + L2 * L2 - c * c) / (2 * L1 * L2)))
    state.gamma = omega - state.alpha - state.beta

    return state


def calc3d(x, y, z, omega):
    r = math.sqrt(x * x + y * y)
    state = calc2d(r, z, omega)
    state.delta = degrees(math.atan(y / x))
    return state


def print_config():
    print("***Robot Arm Config***\n")
    print("* L1=" + str(L1) + LEN_UNIT)
    print("* L2=" + str(L2) + LEN_UNIT)
    print("* L3=" + str(L3) + LEN_UNIT)
    print("* " + str(ALPHA_MIN) + ANGLE_UNIT + " ... alpha ... " + str(ALPHA_MAX) + ANGLE_UNIT)
    print("* " + str(BETA_MIN) + ANGLE_UNIT + " ... beta ... " + str(BETA_MAX) + ANGLE_UNIT)
    print("* " + str(GAMMA_MIN) + ANGLE_UNIT + " ... gamma ... " + str(GAMMA_MAX) + ANGLE_UNIT)
    print("* " + str(DELTA_MIN) + ANGLE_UNIT + " ... delta ... " + str(DELTA_MAX) + ANGLE_UNIT)
    print("* " + str(EPSILON_MIN) + ANGLE_UNIT + " ... epsilon ... " + str(EPSILON_MAX) + ANGLE_UNIT)
    print("* " + str(ZETA_MIN) + ANGLE_UNIT + " ... zeta ... " + str(ZETA_MAX) + ANGLE_UNIT)
    print("**********************\n")


if __name__ == '__main__':
    print_config()
    start = time.perf_counter_ns()
    times = 100000
    for i in range(times):
        state = calc3d(8.2, 8.2, 3.9, 60.0)
    end = time.perf_counter_ns()
    seconds = (end - start) / 1000 / times
    print(f"calc3d() function used {seconds} us\n")
    state.output()
