# 轨迹生成示例 (plan/examples)

本目录为 **轨迹生成与规划** 的集中示例，供动力学验证、重力/负载辨识、摩擦力辨识等应用使用。  
**核心实现** 在上级目录 `plan/`，此处仅为例程。

## 运行前

在项目根目录执行一次：

```matlab
addpaths
```

或手动添加路径：`addpath(genpath('plan/')); addpath(genpath('utility_function/'));`

## 示例列表

| 脚本 | 说明 |
|------|------|
| `example_generate_urdf_validation_trajectory` | URDF 验证轨迹：Part A 正反转匀速 + Part B 正弦/扫频 |
| `example_generate_multi_speed_trajectory` | 多速度档位往返轨迹（全关节、moveJ 平滑） |
| `example_generate_all_joints_slow_trajectory` | 各关节依次 Part A 低速轨迹 |
| `example_generate_fourier_excitation` | 傅里叶激励轨迹（动力学辨识 Part B） |
| `example_extract_target_trajectory` | 从录制 CSV 拆取期望轨迹 |
| `example_view_trajectory` | 轨迹 CSV 读取与可视化 |
| `example_read_multi_joint_csv` | 多关节 CSV 读为 q/qd/qdd（左右腿各 6 列） |

## 输出目录

生成文件默认写入 **build/plan/**，可由各函数参数 `output_file` / `output_dir` 指定。

## 详细文档

- **plan/README.md**：plan 目录总览与函数表  
- **plan/URDF验证轨迹生成说明.md**：Part A/B 原理、参数与测试流程
