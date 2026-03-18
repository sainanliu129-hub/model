# TLBO 激励轨迹方案（E1）

本文件夹为**新增**激励轨迹方案，与上层目录中基于 `generate_excitation_trajectory` + `fmincon` 的方案**并列**，不修改原有生成流程。

**与 `run_excitation_trajectory_standalone.m` 完全一致**（便于对比两种方案效果）：
- **原始机器人**：同一 URDF（E1.urdf）、同一肢体 `get_e1_limb_robot(limb)`，扭矩限值 `max_effort` 同源（URDF 或 50）。
- **约束**：standalone 已设 `config.use_full_bounds_for_optim = true`，与 TLBO 同套限位——关节位置 ±2π（或可选紧限位）、速度 `[16.75; 20.1; …]`、加速度 `[200; …]`，周期 8 s、采样 100 Hz。
- **校核**：TLBO 优化结束后用 `validate_trajectory_E1` 做与 standalone 相同的校核——位置/速度/加速度/扭矩（0.9×max_effort），最多 400 点，位置放宽 1.05 倍。
- **条件数**：统一为方案 B（列归一化 + rref + κ(W_min)=sqrt(cond(W_min'*W_min))），与 cond_fun_from_regressor / cond_value_E1_limb 一致。

## 思路来源

与 **AnaAp.PhyConst.ExcitOptimi.DynIDen** 一致：

- **轨迹参数化**：每关节 11 个傅里叶系数（4 次谐波 a1,b1,…,a4,b4 + q_ini, dq_ini, ddq_ini），由 `Exciting_Trajectory` 生成 q, dq, ddq。
- **约束处理**：`Tradeoff_Modify` 对每关节系数做缩放与平移，使单关节轨迹满足位置/速度/加速度限位。
- **优化目标**：最小化辨识观测矩阵 **条件数** cond(H)，H 由动力学回归矩阵在采样时刻堆叠得到。
- **优化算法**：**TLBO**（Teaching-Learning-Based Optimization）— 教师相（向当前最优学习）+ 学员相（随机两人互学），每步后对系数做 Tradeoff_Modify 再评估目标。

## 与 AnaAp 的差异

| 项目       | AnaAp（CO605）           | 本方案（E1）                    |
|------------|--------------------------|---------------------------------|
| 回归矩阵   | Base_Dynamics_Para_CO605 | ReMatrix_E1_limb_URDF           |
| 肢体/自由度| 6 轴机身 或 4 轴负载     | E1 单肢：腿 6 轴 / 臂 3 轴      |
| 约束惩罚   | Heuristic TLBO 可选      | 可选 `use_constraint_penalty`   |

## 文件说明

| 文件 | 说明 |
|------|------|
| `run_TLBO_Excitation_E1.m` | **入口脚本**：与 standalone 同套机器人/约束/校核，TLBO 优化并绘图。 |
| `validate_trajectory_E1.m` | 校核逻辑与 standalone 的 validate_trajectory 一致（位置/速度/加速度/扭矩）。 |
| `Exciting_Trajectory.m`   | 傅里叶轨迹：系数 → q, dq, ddq（与 AnaAp 公式一致）。 |
| `Tradeoff_Modify.m`      | 单关节系数缩放/平移，满足位置/速度/加速度限位。 |
| `Objective_E1_limb.m`     | 目标函数：调用 cond_value_E1_limb（方案 B）。 |
| `Constraints_Violation_E1.m` | 约束违反量（可选，用于惩罚项）。 |
| `README.md`              | 本说明。 |

## 使用方法

1. 在 MATLAB 中打开 `run_TLBO_Excitation_E1.m`，或 `cd` 到本目录后运行：
   ```matlab
   run_TLBO_Excitation_E1
   ```
2. 在脚本内修改 `limb`（`'left_leg'` / `'right_leg'` / `'left_arm'` / `'right_arm'`）和关节限位、`Population`、`Iteration` 等。
3. 运行后得到当前最优系数 `Best_Solution`、每关节系数矩阵 `Coefficient_ExTra`，以及条件数收敛曲线和单周期轨迹图。

## 依赖

- **ReMatrix_E1_limb_URDF**、**get_e1_limb_robot**：由脚本中 `addpath(body_iden_dir)` 与 `ensure_body_gravity_para_iden_path()` 保证路径（Body_GravityPara_Iden、robot_model、dynamics、utility_function）。
