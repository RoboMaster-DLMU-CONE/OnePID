#ifndef ONE_PID_PIDPARAMS_HPP_
#define ONE_PID_PIDPARAMS_HPP_

#include "Arithmetic.hpp"

#include <limits>
namespace one::pid {

/**
 * @brief PID控制器的参数。
 * @tparam ValueType_ PID计算所使用的数据类型。
 */
template <Arithmetic ValueType_ = float> struct PidParams {
    ValueType_ Kp{}; ///< 比例增益
    ValueType_ Ki{}; ///< 积分增益
    ValueType_ Kd{}; ///< 微分增益
    ValueType_ MaxOutput{
        std::numeric_limits<ValueType_>::infinity()}; ///< 最大输出值

    ValueType_ Deadband{}; ///< 死区范围
    ValueType_ IntegralLimit{
        std::numeric_limits<ValueType_>::infinity()}; ///< 积分项限幅
    ValueType_ DerivativeFilterRC{}; ///< 微分项滤波器的时间常数 (RC)
    ValueType_ OutputFilterRC{};     ///< 输出滤波器的时间常数 (RC)
};

/**
 * @brief 位置式PID算法标签。
 */
struct Positional {};

/**
 * @brief 增量式PID算法标签。
 */
struct Incremental {};

// 特性标签
/** @brief 死区特性标签。如果误差的绝对值小于死区值，则PID输出保持不变。 */
struct WithDeadband // 死区
{};

/** @brief 积分限幅特性标签。限制积分项的累积范围。 */
struct WithIntegralLimit // 积分限幅
{};

/** @brief 微分先行特性标签 (Derivative on
 * Measurement)。微分项基于测量值的变化率，而不是误差的变化率。 */
struct WithDerivativeOnMeasurement // 微分先行
{};

/** @brief 微分项滤波特性标签。为微分项添加一个低通滤波器。 */
struct WithDerivativeFilter // 微分项滤波
{};

/** @brief 输出滤波特性标签。为最终输出添加一个低通滤波器。 */
struct WithOutputFilter // 输出滤波
{};

/** @brief 输出限幅特性标签。限制PID控制器的最终输出范围。 */
struct WithOutputLimit // 输出限幅
{};

} // namespace one::pid

#endif // ONE_PID_PIDPARAMS_HPP_
