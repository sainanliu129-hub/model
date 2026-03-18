# plan — 轨迹与规划

本目录集中存放**轨迹生成与规划**相关代码，供动力学验证、重力/负载辨识、摩擦力辨识等应用统一调用，避免重复实现。

## 目录结构

- **plan/** — 核心函数（本目录根下）
- **plan/examples/** — 轨迹生成示例脚本（见 `plan/examples/README.md`）

## 主要函数

| 函数 | 说明 |
|------|------|
| `generate_urdf_validation_trajectory` | URDF 动力学验证轨迹：Part A 正反转匀速、Part B 正弦/扫频 |
| `generate_fourier_excitation_trajectory` | 傅里叶级数激励轨迹（动力学参数辨识） |
| `generate_multi_speed_trajectory` | 多速度档位往返匀速轨迹（全关节、moveJ 平滑） |
| `generate_constant_velocity_trajectory` | 单关节匀速轨迹（摩擦力辨识等） |
| `generate_friction_trajectory` | 摩擦力辨识用轨迹 |
| `generate_joint_trajectory` | 单关节多档匀速/多档加速度轨迹（MOVEJ） |
| `movej_segment` | 单段 MOVEJ 轨迹（5 次多项式，位置/速度/加速度连续） |
| `extract_target_trajectory_from_csv` | 从录制 CSV 拆取期望轨迹 |
| `read_leg_trajectory_from_csv` | 从 CSV 读取关节数据，仅保留腿部 12 关节并由 30Hz 插值到 500Hz，输出格式同 generate_e1_test_trajectories |
| `visualize_trajectory` | 轨迹 CSV 可视化 |
| `plot_constant_velocity_trajectory` | 匀速轨迹绘图 |

## 使用与输出目录

在 MATLAB 中先执行 **`addpaths`**（项目根目录），或 `addpath(genpath('plan/')); addpath(genpath('utility_function/'));`，即可在任意目录调用上述函数。

**生成文件**：默认写入 **build/plan/**；可通过参数 `output_file`、`output_dir` 指定其他路径。

## 文档

- **URDF 验证轨迹说明**：`plan/URDF验证轨迹生成说明.md`（Part A/B 原理、参数与测试流程）
- **示例说明**：`plan/examples/README.md`

## 依赖

- **utility_function**：`get_build_dir`、`get_data_dir` 等（由 `addpaths` 一并加入）
- URDF 文件：默认查找 `noetix_description/urdf/E1.urdf`（可从工作目录或相对 plan 的上级目录）
- 无其他 toolbox 硬依赖
