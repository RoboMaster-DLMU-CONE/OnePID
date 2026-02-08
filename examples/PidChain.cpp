
#include <one/PID/PidChain.hpp>
#include <one/PID/PidController.hpp>
#include <one/PID/PidParams.hpp>

#include <iostream>

using namespace one::pid;

int main() {
    PidParams<float> p1{.Kp = 1.0, .MaxOutput = 100};
    PidParams<float> p2{.Kp = 2.0, .Ki = 0.5};

    PidController<Positional, float, WithDeadband> pid1(p1);
    PidController<Positional, float, WithOutputFilter> pid2(p2);
    PidChain<float> chain;
    chain.add(pid1);
    chain.add(pid2);

    float output = chain.compute(10.0f, {2.0f, 5.0f});
    std::cout << output << std::endl;

    return 0;
}
