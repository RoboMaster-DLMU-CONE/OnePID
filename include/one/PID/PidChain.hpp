#ifndef ONE_PID_PIDCHAIN_HPP_
#define ONE_PID_PIDCHAIN_HPP_

#include <cstddef>
namespace one::pid {
template <typename Config, typename... RestConfigs> class PidChain {
  using Node = Config::ControllerType;
  using NextChain = PidChain<RestConfigs...>;

public:
  static constexpr size_t Size = sizeof...(RestConfigs) + 1;
  constexpr size_t size() { return Size; }

  explicit PidChain(const Config &conf, const RestConfigs &...rest)
      : head(conf), tail(rest...) {}

  template <typename T, typename... Ms>
  auto compute(T target, T measure, Ms... measures) {
    auto current_output = head.compute(target, measure);
    return tail.compute(current_output, measures...);
  }

  void reset() {
    head.reset();
    tail.reset();
  }

  template <size_t Index> auto &get() {
    if constexpr (Index == 0)
      return head;
    else
      return tail.template get<Index - 1>();
  }

private:
  Node head;
  NextChain tail;
};

template <typename Config> class PidChain<Config> {
  using Node = Config::ControllerType;

public:
  static constexpr size_t Size = 1;
  constexpr size_t size() { return Size; }

  explicit PidChain(const Config &conf) : head(conf) {}

  template <typename T> auto compute(T target, T measure) {
    return head.compute();
  }

  void reset() { return head.reset(); }

  template <size_t Size> auto &get() {
    static_assert(Size == 0, "Index out of bounds");
    return head;
  }

private:
  Node head;
};

} // namespace one::pid

#endif // ONE_PID_PIDCHAIN_HPP_
