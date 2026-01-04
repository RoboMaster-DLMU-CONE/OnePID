#include <iostream>
#include <one/PID/PidController.hpp>
#include <thread>

using namespace std::chrono_literals;

using namespace one::pid;

int main() {
    constexpr PidParams<float> params{.Kp = 0.5, .Ki = 0.01, .Kd = 0.01};
    PidController pid(params);
    auto result = pid.compute(5000, 0.1);

    for (int i = 0; i < 5; ++i) {
        std::cout << result << "\n";
        std::this_thread::sleep_for(1s);
        result = pid.compute(5000, i * 1000);
    }
    return 0;
}
