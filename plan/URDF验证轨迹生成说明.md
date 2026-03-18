# URDF动力学参数验证轨迹生成说明

## 1. 概述

本工具用于生成URDF动力学参数验证测试轨迹，包括两个部分：

- **Part A**: 正反转匀速轨迹（用于重力验证）
- **Part B**: 带加速度轨迹（用于完整动力学力矩验证）

实现与示例位置：**plan/**（函数）、**plan/examples/**（示例脚本）。

## 2. 测试原理

### Part A：重力验证（正反转匀速）

在匀速运动时（$\ddot{q} \approx 0$），单关节正转与反转在同一角度 $\theta$ 处，若摩擦对称，则：

$$\frac{(I_+ + I_-) K_\tau}{2} \approx g(q)$$

其中：
- $I_+$、$I_-$ 为正、反转电流
- $K_\tau$ 为该关节力矩系数
- $g(q)$ 为该关节重力力矩

用URDF算得的理论重力项 $\tau_{\text{gravity}}(q)$ 与上式左侧对比，即可在含摩擦情况下检验重力相关参数（质量、质心）是否准确。

### Part B：动力学力矩验证（带加速度）

带加速度轨迹下，URDF逆动力学给出理论力矩：

$$\bm{\tau}_{\text{model}} = \bm{M}(\bm{q})\ddot{\bm{q}} + \bm{C}(\bm{q},\dot{\bm{q}})\dot{\bm{q}} + \bm{g}(\bm{q})$$

定义残差 $\bm{e} = \bm{\tau}_{\text{real}} - \bm{\tau}_{\text{model}}$，则：

$$\bm{e} = \bm{\tau}_f(\dot{\bm{q}}) + \bm{\varepsilon}_{\text{dyn}}$$

若URDF参数准确，则 $\bm{\varepsilon}_{\text{dyn}} \approx \bm{0}$，残差 $\bm{e}$ 主要随 $\dot{\bm{q}}$ 变化（像摩擦）；若 $\bm{e}$ 明显随 $\bm{q}$ 变化，则说明动力学参数不准。

## 3. 使用方法

### 3.1 基本用法

```matlab
% 生成Part A轨迹（正反转匀速）
generate_urdf_validation_trajectory('part_a', 'l_leg_hip_yaw_joint', ...
    'velocity', 0.2, 'angle_range', 2*pi);

% 生成Part B轨迹（正弦）
generate_urdf_validation_trajectory('part_b', 'l_leg_hip_yaw_joint', ...
    'trajectory_shape', 'sine', 'amplitude', 0.5, 'frequency', 0.1, 'duration', 30);
```

### 3.2 运行示例脚本

```matlab
addpaths   % 项目根目录执行一次
example_generate_urdf_validation_trajectory   % 在 plan/examples 中
```

## 4. 函数参数说明

### 4.1 必需参数

- `trajectory_type`: 轨迹类型
  - `'part_a'`: Part A轨迹（正反转匀速）
  - `'part_b'`: Part B轨迹（带加速度）

- `joint_name`: 要测试的关节名称，支持的关节：
  - `'l_leg_hip_yaw_joint'`
  - `'l_leg_hip_roll_joint'`
  - `'l_leg_hip_pitch_joint'`
  - `'l_leg_knee_joint'`
  - `'l_leg_ankle_pitch_joint'`
  - `'l_leg_ankle_roll_joint'`
  - `'r_leg_hip_yaw_joint'`
  - `'r_leg_hip_roll_joint'`
  - `'r_leg_hip_pitch_joint'`
  - `'r_leg_knee_joint'`
  - `'r_leg_ankle_pitch_joint'`
  - `'r_leg_ankle_roll_joint'`

### 4.2 可选参数

#### Part A参数

- `'Ts'`: 采样周期，默认 `0.002` s (500 Hz)
- `'velocity'`: 匀速速度 rad/s，默认 `0.2`
- `'angle_range'`: 角度范围 rad，默认 `2*pi`（1圈）
  - **如果提供 `'urdf_file'` 参数，将自动从URDF读取关节限位并设置 `angle_range`**
  - 自动读取时，使用限位范围的90%作为安全余量
- `'urdf_file'`: URDF文件路径（可选）
  - 如果未指定，Part A会自动尝试查找URDF文件（从当前目录向上查找）
  - 如果找到URDF文件，将自动读取关节限位并设置 `angle_range`

#### Part B参数

- `'Ts'`: 采样周期，默认 `0.002` s (500 Hz)
- `'trajectory_shape'`: 轨迹形状
  - `'sine'`: 正弦轨迹（默认）
  - `'chirp'`: 扫频轨迹
- `'amplitude'`: 幅值 rad，默认 `0.5`
- `'frequency'`: 频率 Hz（sine用），默认 `0.1`
- `'freq_start'`: 扫频起始频率 Hz，默认 `0.05`
- `'freq_end'`: 扫频结束频率 Hz，默认 `0.5`
- `'duration'`: 持续时间 s，默认 `30`
- `'output_file'`: 输出文件名，默认自动生成（格式：`{joint_name}_part_{a|b}_{timestamp}.csv`）

## 5. 输出格式

生成的CSV文件格式：

```csv
time,l_leg_hip_yaw_joint,l_leg_hip_roll_joint,...,r_leg_ankle_roll_joint
0.00000,0.000000,0.000000,...,0.000000
0.00200,0.000400,0.000000,...,0.000000
...
```

- 只有目标关节有运动，其他关节保持为0
- 默认输出目录：**build/plan/**

## 6. Part A 轨迹说明

### 6.1 轨迹结构

1. **正转段**：从位置0匀速运动到 `angle_range`（速度 `+velocity`）
2. **回程段**：从 `angle_range` 匀速回到0（速度 `-velocity`），**此段不参与计算**
3. **反转段**：从位置0匀速运动到 `-angle_range`（速度 `-velocity`）

### 6.2 使用建议

- **速度选择**：建议 `0.1 ~ 0.3` rad/s
- **角度范围**：推荐使用 `urdf_file` 从URDF自动读取限位（90%安全余量）
- **采样频率**：默认 500 Hz

## 7. Part B 轨迹说明

### 7.1 轨迹类型

- **正弦（sine）**：$q(t) = A \sin(2\pi f t)$
- **扫频（chirp）**：幅值 A，频率由 `freq_start` 线性扫到 `freq_end`

### 7.2 使用建议

- 幅值建议 `0.3 ~ 0.8` rad；正弦频率 `0.05 ~ 0.5` Hz；持续时间建议 `20 ~ 40` s

## 8. 测试流程

1. **先做 Part A**（重力验证），再**做 Part B**（动力学力矩验证）
2. Part A 不通过 → 优先查质量、质心或 $K_\tau$
3. Part A 通过且 Part B 中 $e$ 随 $q$ 变化明显 → 重点查惯量或 $K_\tau$

## 9. 文件说明

- **plan/generate_urdf_validation_trajectory.m**：主函数
- **plan/examples/example_generate_urdf_validation_trajectory.m**：示例脚本
- **plan/URDF验证轨迹生成说明.md**：本文档
