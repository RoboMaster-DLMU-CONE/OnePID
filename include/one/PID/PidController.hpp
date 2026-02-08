
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

/**
 * @brief 特性包结构体，用于组合多个特性
 * @tparam Fs 可变参数模板，代表多个特性
 */
template <typename... Fs> struct FeaturePack {};

namespace detail {
/**
 * @brief 检查目标特性是否存在于类型T中
 * @tparam Target 目标特性类型
 * @tparam T 被检查的类型
 */
template <typename Target, typename T>
struct HasFeatureCheck : std::is_same<Target, T> {};

/**
 * @brief 检查目标特性是否存在于特性包中
 * @tparam Target 目标特性类型
 * @tparam Fs 特性包中的特性类型
 */
template <typename Target, typename... Fs>
struct HasFeatureCheck<Target, FeaturePack<Fs...>> {
    static constexpr bool value = (HasFeatureCheck<Target, Fs>::value || ...);
};
} // namespace detail

/**
 * @brief PID节点基类，定义了PID控制器的基本接口
 * @tparam ValueType 数值类型
 */
template <Arithmetic ValueType> class PidNode {
  public:
    virtual ~PidNode() = default;
    
    /**
     * @brief 使用给定时间步长计算PID输出
     * @param ref 参考值
     * @param measure 测量值
     * @param dt 时间步长
     * @return 计算得到的PID输出
     */
    virtual ValueType computeWithDt(ValueType ref, ValueType measure,
                                    ValueType dt) = 0;
    
    /**
     * @brief 重置PID控制器内部状态
     */
    virtual void reset() = 0;
};

/**
 * @brief 可配置的PID控制器类
 * @tparam Algorithm PID算法类型（位置式或增量式）
 * @tparam ValueType 数值类型，必须满足Arithmetic概念
 * @tparam Features 可选的特性包，用于启用特定功能
 * @details 实现了完整的PID控制算法，支持多种算法变体和可配置特性
 */
template <typename Algorithm = Positional, Arithmetic ValueType = float,
          typename... Features>
class PidController : public PidNode<ValueType> {
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

    ValueType ITerm{}; ///< 积分项累加值
    ValueType prev_error{}; ///< 上一次的误差值
    ValueType prev_prev_error{}; ///< 上上次的误差值
    ValueType prev_ref{}; ///< 上一次的参考值
    ValueType prev_measure{}; ///< 上一次的测量值
    ValueType prev_derivative{}; ///< 上一次的微分值
    ValueType prev_output{}; ///< 上一次的输出值
    DeltaT<ValueType> deltaT{}; ///< 用于计算时间增量的工具对象

  public:
    using ParamsType = PidParams<ValueType>; ///< 参数类型别名
    
    mutable ValueType MaxOutputVal; ///< 最大输出值限制
    mutable ValueType DeadbandVal; ///< 死区范围
    mutable ValueType IntegralLimitVal; ///< 积分项限幅值
    mutable ValueType Kp; ///< 比例增益
    mutable ValueType Ki; ///< 积分增益
    mutable ValueType Kd; ///< 微分增益
    mutable ValueType D_RC; ///< 微分项滤波器时间常数
    mutable ValueType O_RC; ///< 输出滤波器时间常数

    /**
     * @brief 构造函数，使用PID参数初始化控制器
     * @param params PID参数结构体
     */
    explicit PidController(const PidParams<ValueType> &params)
        : MaxOutputVal(params.MaxOutput), DeadbandVal(params.Deadband),
          IntegralLimitVal(params.IntegralLimit), Kp(params.Kp), Ki(params.Ki),
          Kd(params.Kd), D_RC(params.DerivativeFilterRC),
          O_RC(params.OutputFilterRC) {}

    /**
     * @brief 计算PID控制器的输出
     * @tparam useBuiltinDeltaT 是否使用内置的时间增量计算，默认为true
     * @param ref 参考值（期望值）
     * @param measure 测量值（实际值）
     * @param dt 时间步长，当useBuiltinDeltaT为false时使用
     * @return PID控制器的输出值
     * @details 该函数实现了完整的PID算法，包括比例、积分、微分项的计算，
     *          并根据配置的特性进行死区处理、积分限幅、微分滤波等操作
     */
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

    /**
     * @brief 使用给定时间步长计算PID输出（重写基类方法）
     * @param ref 参考值
     * @param measure 测量值
     * @param dt 时间步长
     * @return PID控制器的输出值
     */
    ValueType computeWithDt(ValueType ref, ValueType measure,
                            ValueType dt) override {
        return compute<false>(ref, measure, dt);
    }

    /**
     * @brief 重置PID控制器的所有内部状态
     * @details 将积分项、历史误差、历史输出等状态变量重置为初始值
     */
    void reset() override {
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
