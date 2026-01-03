# OnePID

一个现代C++20 PID控制器库，设计用于嵌入式系统和实时控制应用。

## 特性

- **C++20 概念约束** - 利用C++20概念系统确保类型安全
- **灵活的算法支持** - 支持多种PID算法（位置式、增量式等）
- **可配置的参数** - 完整的PID参数配置（Kp, Ki, Kd等）
- **高级特性**
  - 微分项滤波
  - 输出滤波
  - 积分限幅
  - 死区设置
  - 最大输出限制
- **链式控制器** - 支持多级串联PID控制器（PidChain）
- **自动时间管理** - DeltaT工具类自动计算时间增量
- **嵌入式友好** - 支持Zephyr RTOS和标准C++环境

## 文件结构

```
OnePID/
├── include/one/PID/
│   ├── Arithmetic.hpp           # C++20算术类型概念
│   ├── DeltaT.hpp               # 时间增量计算工具
│   ├── PidController.hpp        # 主PID控制器实现
│   ├── PidParams.hpp            # PID参数定义
│   ├── PidConfig.hpp            # PID配置模板
│   └── PidChain.hpp             # 链式控制器支持
├── examples/
│   └── PIDController.cpp        # 使用示例
├── CMakeLists.txt              # CMake构建配置
└── README.md                   # 本文件
```

## 核心组件

### Arithmetic.hpp
定义了`Arithmetic`概念，约束模板参数必须为整数或浮点数类型。

### DeltaT.hpp
时间增量计算工具类，自动计算两次调用之间的时间差（以毫秒为单位）。支持自定义数据类型。

```cpp
DeltaT<float> timer;
float deltaTime = timer.getDeltaMS();
```

### PidParams.hpp
PID控制器的完整参数配置：

| 参数 | 说明 |
|------|------|
| `Kp` | 比例增益 |
| `Ki` | 积分增益 |
| `Kd` | 微分增益 |
| `MaxOutput` | 最大输出值限制 |
| `Deadband` | 死区范围 |
| `IntegralLimit` | 积分项限幅 |
| `DerivativeFilterRC` | 微分项滤波器时间常数 |
| `OutputFilterRC` | 输出滤波器时间常数 |

### PidController.hpp
主PID控制器类，支持：
- 多种PID算法（通过模板参数指定）
- 任意数据类型（整数或浮点数）
- 动态特性配置

### PidConfig.hpp
为PID控制器提供配置接口，简化参数管理。

### PidChain.hpp
支持多级串联PID控制器，用于复杂的控制系统。

## 编译和使用

### 最小配置

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### 启用示例

```bash
mkdir build
cd build
cmake -DONE_PID_BUILD_EXAMPLES=ON ..
cmake --build .
```

### 启用单元测试

```bash
cmake -DONE_PID_BUILD_TESTS=ON ..
cmake --build .
```

### 构建要求

- CMake >= 3.25
- C++20 兼容编译器（GCC 10+, Clang 10+, MSVC 2019+）

## 使用示例

```cpp
#include <one/PID/PidController.hpp>
#include <iostream>

using namespace one::pid;

int main() {
    // 创建PID参数
    constexpr PidParams<float> params{
        .Kp = 0.5,
        .Ki = 0.01,
        .Kd = 0.01
    };
    
    // 创建PID控制器
    PidController<Positional, float> pid(params);
    
    // 计算输出
    float target = 5000.0f;
    float measurement = 0.0f;
    float output = pid.compute(target, measurement);
    
    std::cout << "PID Output: " << output << std::endl;
    
    return 0;
}
```

### 链式控制器示例

```cpp
PidConfig<Positional, float> config1(params1);
PidConfig<Positional, float> config2(params2);

PidChain<decltype(config1), decltype(config2)> chain(config1, config2);

// 使用链式控制器
auto output = chain.compute(target1, measure1, measure2);
```

## Zephyr RTOS 支持

此库完全支持Zephyr RTOS开发环境。CMake构建系统会自动检测Zephyr工具链并使用相应的配置。

## 许可证

本项目采用ISC开源许可证。详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request。
