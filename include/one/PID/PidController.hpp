
/**
 * @file PID.hpp
 * @brief 定义了一个可配置的PID控制器。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef PID_HPP
#define PID_HPP
#include "Arithmetic.hpp"
#include "DeltaT.hpp"
#include "PidParams.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <type_traits>

namespace one::pid {

template <typename... Fs> struct FeaturePack {};

namespace detail {
template <typename Target, typename T>
struct HasFeatureCheck : std::is_same<Target, T> {};

template <typename Target, typename... Fs>
struct HasFeatureCheck<Target, FeaturePack<Fs...>> {
    static constexpr bool value = (HasFeatureCheck<Target, Fs>::value || ...);
};
} // namespace detail

template <typename Algorithm = Positional, Arithmetic ValueType = float,
          typename... Features>
class PidController {
    template <typename TargetFeature>
    static constexpr bool Check =
        (detail::HasFeatureCheck<TargetFeature, Features>::value || ...);

    static constexpr bool PositionalPID = std::is_same_v<Algorithm, Positional>;
    static constexpr bool IncrementalPID = !PositionalPID;

    static constexpr bool HasDeadband = Check<WithDeadband>;
    static constexpr bool HasIntegralLimit = Check<WithIntegralLimit>;
    static constexpr bool HasDerivativeOnMeasurement =
        Check<WithDerivativeOnMeasurement>;
    static constexpr bool HasDerivativeFilter = Check<WithDerivativeFilter>;
    static constexpr bool HasOutputFilter = Check<WithOutputFilter>;
    static constexpr bool HasOutputLimit = Check<WithOutputLimit>;

    ValueType ITerm{};
    ValueType prev_error{};
    ValueType prev_prev_error{};
    ValueType prev_ref{};
    ValueType prev_measure{};
    ValueType prev_derivative{};
    ValueType prev_output{};
    DeltaT<ValueType> deltaT{};

  public:
    using ParamsType = PidParams<ValueType>;
    mutable ValueType MaxOutputVal;
    mutable ValueType DeadbandVal;
    mutable ValueType IntegralLimitVal;
    mutable ValueType Kp;
    mutable ValueType Ki;
    mutable ValueType Kd;
    mutable ValueType D_RC;
    mutable ValueType O_RC;

    explicit PidController(const PidParams<ValueType> &params)
        : MaxOutputVal(params.MaxOutput), DeadbandVal(params.Deadband),
          IntegralLimitVal(params.IntegralLimit), Kp(params.Kp), Ki(params.Ki),
          Kd(params.Kd), D_RC(params.DerivativeFilterRC),
          O_RC(params.OutputFilterRC) {}

    template <bool useBuiltinDeltaT = true>
    ValueType compute(ValueType ref, ValueType measure, ValueType dt = 1) {
        // 1. 获取时间增量
        if constexpr (useBuiltinDeltaT) {
            dt = deltaT.getDeltaMS();
        } else {
            assert(dt > 0);
        }

        const ValueType error = ref - measure;

        // 2. 死区处理 (保持上次输出)
        if constexpr (HasDeadband) {
            if (std::abs(error) <= DeadbandVal) {
                return prev_output;
            }
        }

        ValueType P, I, D, output;

        // 3. 计算 P 项
        if constexpr (PositionalPID) {
            P = Kp * error;
        } else {
            // 增量式：P 是 delta P
            P = Kp * (error - prev_error);
        }

        // 4. 计算 I 项
        if constexpr (PositionalPID) {
            // 位置式：积分累加
            ITerm += Ki * error * dt;

            // 积分限幅 (Anti-windup)
            if constexpr (HasIntegralLimit) {
                ITerm = std::clamp(ITerm, -IntegralLimitVal, IntegralLimitVal);
            }
            I = ITerm;
        } else {
            // 增量式：I 是 delta I = Ki * error * dt
            // 增量式通常不需要累积
            // ITerm，除非是为了输出限幅后的反计算，这里简化处理
            I = Ki * error * dt;
        }

        // 5. 计算 D 项
        D = [&] {
            if constexpr (HasDerivativeOnMeasurement) {
                // 微分先行 (对测量值求导，避免目标值突变冲击)
                // 注意：在增量式下混合使用微分先行需要仔细定义物理意义
                // 这里统一处理为由测量值变化引起的 D 项分量
                ValueType meas_deriv = -(measure - prev_measure) / dt;
                return meas_deriv;
            } else if constexpr (PositionalPID) {
                // 位置式 D = de/dt
                return (error - prev_error) / dt;
            } else {
                // 增量式 D = delta D = (e(k) - 2e(k-1) + e(k-2)) / dt
                return (error - 2 * prev_error + prev_prev_error) / dt;
            }
        }();
        D *= Kd;

        // 微分滤波
        if constexpr (HasDerivativeFilter) {
            // 简单的低通滤波: y = a*x + (1-a)*y_prev
            // alpha = dt / (RC + dt)
            D = D * dt / (D_RC + dt) + prev_derivative * D_RC / (D_RC + dt);
        }

        // 6. 计算总输出
        if constexpr (PositionalPID) {
            output = P + I + D;
        } else {
            output = prev_output + P + I + D;
        }

        // 7. 输出滤波
        if constexpr (HasOutputFilter) {
            output =
                output * dt / (O_RC + dt) + prev_output * O_RC / (O_RC + dt);
        }

        // 8. 输出限幅
        if constexpr (HasOutputLimit) {
            output = std::clamp(output, -MaxOutputVal, MaxOutputVal);
        }

        // 9. 更新状态变量
        prev_ref = ref;
        prev_measure = measure;
        prev_derivative = D;
        prev_output = output;

        prev_prev_error = prev_error;
        prev_error = error;

        return output;
    }

    void reset() {
        ITerm = ValueType{};
        prev_error = ValueType{};
        prev_prev_error = ValueType{};
        prev_ref = ValueType{};
        prev_measure = ValueType{};
        prev_derivative = ValueType{};
        prev_output = ValueType{};
        deltaT.reset();
    }
};
} // namespace one::pid

#endif // PID_HPP
