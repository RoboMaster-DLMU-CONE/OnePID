
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidController.hpp>

using namespace std::chrono_literals;

using namespace one::pid;

int main() {
    constexpr PidParams<float> p1{.Kp = 1.0, .MaxOutput = 100};
    constexpr PidParams<float> p2{.Kp = 2.0, .Ki = 0.5};

    constexpr PidConfig<Positional, float, WithDeadband> conf1(p1);
    constexpr PidConfig<Positional, float, WithOutputFilter> conf2(p2);
    PidChain chain(conf1, conf2);

    float output = chain.compute(10.0f, 2.0f, 5.0f);

    return 0;
}
