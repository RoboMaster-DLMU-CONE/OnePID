#ifndef ONE_PID_PIDCHAIN_HPP_
#define ONE_PID_PIDCHAIN_HPP_

#include "DeltaT.hpp"
#include "PidController.hpp"
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <span>
#include <vector>

namespace one::pid {

/**
 * @brief 串联多个PID控制器的链式控制器类
 * @tparam ValueType 计算使用的数值类型，必须满足Arithmetic概念
 * @details PidChain允许将多个PID控制器串联起来形成一个控制链，
 *          每个控制器的输出作为下一个控制器的输入
 */
template <Arithmetic ValueType = float> class PidChain {
  public:
    /**
     * @brief 默认构造函数
     */
    PidChain() = default;

    /**
     * @brief 使用初始化列表构造链式控制器
     * @param nodes PID节点指针的初始化列表
     */
    explicit PidChain(std::initializer_list<PidNode<ValueType> *> nodes)
        : nodes_(nodes) {}

    /**
     * @brief 添加一个PID节点到链中
     * @param node 要添加的PID节点引用
     */
    void add(PidNode<ValueType> &node) { nodes_.push_back(&node); }

    /**
     * @brief 清空PID链中的所有节点
     */
    void clear() { nodes_.clear(); }

    /**
     * @brief 获取链中PID节点的数量
     * @return 链中PID节点的数量
     */
    [[nodiscard]] size_t size() const { return nodes_.size(); }

    /**
     * @brief 检查链是否为空
     * @return 如果链中没有PID节点则返回true，否则返回false
     */
    [[nodiscard]] bool empty() const { return nodes_.empty(); }

    /**
     * @brief 使用span形式的测量值计算PID链的输出
     * @param target 目标值
     * @param measures 测量值的span，其大小应与节点数量匹配
     * @return PID链的最终输出值
     */
    ValueType compute(ValueType target, std::span<const ValueType> measures) {
        // assert(!nodes_.empty());
        // assert(measures.size() == nodes_.size());

        const ValueType dt = d.getDeltaMS();
        ValueType output = target;
        for (size_t i = 0; i < nodes_.size(); ++i) {
            // assert(nodes_[i]);
            output = nodes_[i]->computeWithDt(output, measures[i], dt);
        }
        return output;
    }

    /**
     * @brief 使用初始化列表形式的测量值计算PID链的输出
     * @param target 目标值
     * @param measures 测量值的初始化列表，其大小应与节点数量匹配
     * @return PID链的最终输出值
     */
    ValueType compute(ValueType target,
                      std::initializer_list<ValueType> measures) {
        return compute(
            target, std::span<const ValueType>(measures.begin(), measures.size()));
    }

    /**
     * @brief 重置PID链中所有节点的状态
     */
    void reset() {
        for (auto *node : nodes_) {
            // assert(node);
            node->reset();
        }
        d.reset();
    }

  private:
    std::vector<PidNode<ValueType> *> nodes_; ///< 存储PID节点指针的向量
    DeltaT<ValueType> d; ///< 用于计算时间增量的工具对象
};

} // namespace one::pid

#endif // ONE_PID_PIDCHAIN_HPP_
