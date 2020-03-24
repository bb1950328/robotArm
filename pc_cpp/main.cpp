#include <ctime>
#include "robotArm.cpp"

static const int times = 10000;

int main() {
    print_config();
    ServoState state;
    clock_t start = clock();
    for (int i = 0; i < times; ++i) {
        state = calc3d(8.2f, 8.2f, 3.9f, 60.0f);
    }
    clock_t end = clock();
    float seconds = (float) (end - start) * 1000000 / (float)times / CLOCKS_PER_SEC;
    printf("calc3d() function used %.12f us\n", seconds);
    state.print();
    return 0;
}
