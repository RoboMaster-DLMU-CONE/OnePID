#ifndef ONE_PID_PIDCONFIG_HPP_
#define ONE_PID_PIDCONFIG_HPP_

#include "PidController.hpp"
#include "PidParams.hpp"
namespace one::pid {

template <typename Algorithm = Positional, typename ValueType = float,
          typename... Features>
struct PidConfig : public PidParams<ValueType> {
    using ControllerType = PidController<Algorithm, ValueType, Features...>;
    using _ValueType = ValueType;
    constexpr PidConfig(const PidParams<ValueType> &p)
        : PidParams<ValueType>(p) {};
    constexpr PidConfig() = default;
};

} // namespace one::pid

#endif // ONE_PID_PIDCONFIG_HPP_
