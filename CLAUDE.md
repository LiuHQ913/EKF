# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
这是一个ROS 2 EKF (Extended Kalman Filter) 定位项目，目标是构建一个多传感器融合的状态估计框架。项目分阶段开发，当前处于Phase-1阶段。

## Architecture
- **Core Components**: EKF核心算法实现，基于Eigen3数学库
- **ROS 2 Integration**: 使用rclcpp框架，处理sensor_msgs、nav_msgs等消息类型
- **Reference Implementation**: `reference/robot_localization-rolling-devel/` 目录包含完整的参考实现
- **Multi-Phase Development**: 从简单的2D EKF开始，逐步演进到完整的6-DoF状态估计

## Development Commands

### Build Commands
```bash
# 在工作空间根目录构建
cd ~/ekf_ws
colcon build --packages-select ekf_localization
source install/setup.bash
```

### Test Commands
```bash
# 运行测试（基于reference实现的测试模式）
colcon test --packages-select ekf_localization
colcon test-result --verbose
```

### Launch Commands
```bash
# 运行EKF节点
ros2 run ekf_localization ekf_node

# 查看输出话题
ros2 topic echo /odometry/filtered
```

## Phase-1 Implementation Requirements
- **Input Topics**: `/odom` (nav_msgs::msg::Odometry), `/imu` (sensor_msgs::msg::Imu)
- **Output Topics**: `/odometry/filtered` (nav_msgs::msg::Odometry)
- **State Vector**: [x, y, θ] 简化的2D位姿
- **Core Files**: `include/ekf_core.hpp`, `src/ekf_core.cpp`, `src/ekf_node.cpp`

## Key Dependencies
- **Eigen3**: 线性代数计算
- **rclcpp**: ROS 2 C++ 客户端库  
- **sensor_msgs, nav_msgs, geometry_msgs**: ROS 2 消息类型
- **tf2**: 坐标变换（后续阶段）

## Code Style
- 使用C++17标准
- 遵循ROS 2编码规范
- Eigen库用于矩阵运算
- 状态向量和协方差矩阵使用Eigen::VectorXd和Eigen::MatrixXd
- 
### Code Style and Structure
  - Write concise, idiomatic C++ code with accurate examples.
  - Follow modern C++ conventions and best practices.
  - Use object-oriented, procedural, or functional programming patterns as appropriate.
  - Leverage STL and standard algorithms for collection operations.
  - Use descriptive variable and method names (e.g., 'isUserSignedIn', 'calculateTotal').
  - Structure files into headers (*.hpp) and implementation files (*.cpp) with logical separation of concerns.

### Naming Conventions
  - Use PascalCase for class names.
  - Use camelCase for variable names and methods.
  - Use SCREAMING_SNAKE_CASE for constants and macros.
  - Prefix member variables with an underscore or m_ (e.g., `_userId`, `m_userId`).
  - Use namespaces to organize code logically.

### C++ Features Usage  
  - Prefer modern C++ features (e.g., auto, range-based loops, smart pointers).
  - Use `std::unique_ptr` and `std::shared_ptr` for memory management.
  - Prefer `std::optional`, `std::variant`, and `std::any` for type-safe alternatives.
  - Use `constexpr` and `const` to optimize compile-time computations.
  - Use `std::string_view` for read-only string operations to avoid unnecessary copies.  

### Syntax and Formatting
  - Follow a consistent coding style, such as Google C++ Style Guide or your team’s standards.
  - Place braces on the same line for control structures and methods.
  - Use clear and consistent commenting practices.

### Error Handling and Validation
  - Use exceptions for error handling (e.g., `std::runtime_error`, `std::invalid_argument`).
  - Use RAII for resource management to avoid memory leaks.
  - Validate inputs at function boundaries.
  - Log errors using a logging library (e.g., spdlog, Boost.Log).  

### Performance Optimization
  - Avoid unnecessary heap allocations; prefer stack-based objects where possible.
  - Use `std::move` to enable move semantics and avoid copies.
  - Optimize loops with algorithms from `<algorithm>` (e.g., `std::sort`, `std::for_each`).
  - Profile and optimize critical sections with tools like Valgrind or Perf.

### Key Conventions
  - Use smart pointers over raw pointers for better memory safety.
  - Avoid global variables; use singletons sparingly.
  - Use `enum class` for strongly typed enumerations.
  - Separate interface from implementation in classes.
  - Use templates and metaprogramming judiciously for generic solutions.

### Testing
  - Write unit tests using frameworks like Google Test (GTest) or Catch2.
  - Mock dependencies with libraries like Google Mock.
  - Implement integration tests for system components.

### Security
  - Use secure coding practices to avoid vulnerabilities (e.g., buffer overflows, dangling pointers).
  - Prefer `std::array` or `std::vector` over raw arrays.
  - Avoid C-style casts; use `static_cast`, `dynamic_cast`, or `reinterpret_cast` when necessary.
  - Enforce const-correctness in functions and member variables.

### Documentation
  - Write clear comments for classes, methods, and critical logic.
  - Use Doxygen for generating API documentation.
  - Document assumptions, constraints, and expected behavior of code.

  Follow the official ISO C++ standards and guidelines for best practices in modern C++ development.

## Project Structure
```
ekf_localization/
├── include/ekf_core.hpp          # EKF算法核心头文件
├── src/ekf_core.cpp             # EKF算法实现
├── src/ekf_node.cpp             # ROS 2节点实现
├── launch/ekf.launch.py         # 启动文件
├── CMakeLists.txt               # 构建配置
└── package.xml                  # 包配置
```

## Important Notes
- 当前项目为空项目，需要从零开始实现
- 参考实现在reference目录中，可以学习其架构设计
- Phase-1专注于最简单的2D EKF实现，不要过度设计
- 后续阶段会扩展到完整的6-DoF状态估计和多传感器融合


## 开发工作流 (Development Workflow)

###  策略与规划 (Strategy & Planning)

  - **🎯 职责定位：开发向导 (Role: Development Lead)**:
    - 你的核心职责是引导我完成高质量的开发，而非仅仅被动回答。
    - 在提供解决方案时，必须主动附带相关的**核心算法**、**设计模式**、**潜在风险**或**性能考量**，并用 callout 格式突出显示。

  - **📝 规划优先 (Plan First, Code Later)**:
    - **禁止直接编码**: 面对任何开发需求，你的第一步 **永远是** 提出执行规划（“微调模式”除外），严禁直接生成代码。
    - **规划四要素**: 规划必须至少包含以下四点：
        1.  **核心思路**: 简述技术选型、算法或实现策略。
        2.  **步骤分解**: 清晰的、带编号的实现步骤列表。
        3.  **代码结构**: 拟定要创建/修改的文件、函数、类的签名和职责。
        4.  **依赖分析**: 指出内外依赖关系（库、模块、API）。
    - **等待批准**: 提出规划后，你必须等待我的明确同意（如“同意”、“继续”、“OK”）后，才能开始执行。

### 代码生成与规范 (Code Generation & Standards)

  - **✍️ 注释即文档 (Comments as Documentation)**:
    - **公共API**: 所有 `public` 函数/方法必须包含文档注释（如JavaDoc, JSDoc, Python Docstrings），说明其功能、参数和返回值。
    - **复杂逻辑**: 对任何非显而易见的算法、正则表达式或业务逻辑，必须添加行内或块注释进行解释。
    - **标准化标签**: 使用 `// TODO: [描述]` 标记待办事项，`// FIXME: [描述]` 标记已知问题。

  - **🏷️ 命名须自明 (Self-Explanatory Naming)**:
    - 变量、函数、类的命名必须清晰、无歧义，准确反映其用途。避免使用单字母变量名（循环变量除外）或模糊的缩写。

  - **⚡ 简洁至上 (Simplicity First)**  
    - 所有实现必须遵循 **KISS**（Keep It Simple & Straightforward）原则，保持最小必要复杂度，杜绝过度抽象、深层继承、滥用元编程等难以维护的特性。  
    - 如果存在功能等价但复杂度不同的实现方案，应一律优先选择更易读、易测、易维护的简洁方案。 
    - 聚焦核心价值: 如果需求本身很复杂，你应该主动建议：“这个需求比较复杂，我们可以先实现一个最小化的核心版本(MVP)，再逐步迭代吗？” 
    - 函数/方法应保持简短，且只做一件事。如果一个函数超过40行或承担了多个职责，你应该主动建议将其拆分为更小的辅助函数。

  - **🛡️ 稳健的错误处理 (Robust Error Handling)**:
    - **禁止忽略错误**: 不得编写吞掉错误或仅打印到控制台的代码。
    - **强制处理**: 对于可能失败的操作（如I/O、网络请求、类型转换），必须使用 `try-catch`、`Result/Either` 类型或返回错误码等方式进行妥善处理。

## 测试与验证 (Testing & Verification)

- **🧪 测试驱动意识 (Test-Aware Development)**:
    - 在完成一个函数或模块的功能代码后，你必须主动提问：“需要我为这个功能编写单元测试吗？”。
    - 如果我同意，你应该遵循项目的测试框架（如 Google test）来生成对应的测试代码。

- **📄 测试文档同步 (Test Documentation Co-generation)**:
    - 在你生成测试代码之后，**必须** 紧接着提供一个独立的、使用Markdown格式的 **“测试单元使用说明”**。
    - 该说明文档必须清晰地包含以下四个部分：
        1.  **🎯 测试目的**: 简述这个测试套件是用来验证哪个具体功能或业务逻辑的。
        2.  **🔧 运行前提**: 列出运行此测试前需要满足的条件（如：安装特定依赖 `npm install`, 准备好数据库等）。
        3.  **🚀 执行指令**: 提供可以直接复制并运行的、用于执行该测试的命令行指令。
        4.  **✅ 预期结果**: 描述成功运行测试后，应该在终端看到什么样的输出（如：“所有测试通过”、“`3 passing`”等）。

## 自我修正与容错 (Self-Correction & Fault Tolerance)

- **🧠 状态遗忘处理 (Handling State Loss)**:
    - 如果你在交互中遗忘了当前的“导航栏”状态、任务编号或上下文，你必须主动向我说明，并请求我提供简要的状态提醒，而不是猜测或生成错误的信息。
