#ifndef ONE_PID_PIDCHAIN_HPP_
#define ONE_PID_PIDCHAIN_HPP_

#include "DeltaT.hpp"
#include <cstddef>
namespace one::pid {

namespace detail {
template <typename Config, typename... RestConfigs> class ChainCore {
    using Node = Config::ControllerType;
    using NextCore = ChainCore<RestConfigs...>;

  public:
    static constexpr size_t Size = sizeof...(RestConfigs) + 1;
    constexpr size_t size() { return Size; }

    explicit ChainCore(const Config &conf, const RestConfigs &...rest)
        : head(conf), tail(rest...) {};

    template <typename T, typename... Ms>
    auto compute(T target, T dt, T measure, Ms... measures) {
        auto current_output = head.template compute<true>(target, measure, dt);
        return tail.compute(current_output, dt, measures...);
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
    NextCore tail;
};
template <typename Config> class ChainCore<Config> {
    using Node = Config::ControllerType;

  public:
    static constexpr size_t Size = 1;
    constexpr size_t size() { return Size; }

    explicit ChainCore(const Config &config) : head(config) {};

    template <typename T> auto compute(T target, T dt, T measure) {
        return head.template compute<true>(target, measure, dt);
    }

    void reset() { head.reset(); }

    template <size_t Index> auto &get() {
        static_assert(Index == 0, "Index out of bounds");
        return head;
    }

  private:
    Node head;
};

} // namespace detail

template <typename... Configs> class PidChain {

  public:
    explicit PidChain(const Configs &...configs) : core(configs...) {}

    template <typename T, typename... Ms>
    auto compute(T target, T measure, Ms... measures) {
        return core.compute(target, d.getDeltaMS(), measure, measures...);
    }

    void reset() { core.reset(); }

    template <size_t Index> auto &get() { return core.template get<Index>(); }

  private:
    detail::ChainCore<Configs...> core;
    DeltaT<float> d;
};

} // namespace one::pid

#endif // ONE_PID_PIDCHAIN_HPP_
