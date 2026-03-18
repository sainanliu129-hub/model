# E1 动力学计算说明（按肢体分开算）

## 概述

E1 为双腿+双臂 18 自由度机器人，**推荐按肢体单独计算动力学**：每条腿 6 维、每条臂 3 维。关节角/角速度/角加速度及质量矩阵 M、科氏项 C、重力项 G 的维度均为**当前肢体自由度**。

- **左腿 / 右腿**：各 6 自由度，`q/qd/qdd` 与 `tau、M、C、G` 均为 6×1 或 6×6。
- **左臂 / 右臂**：各 3 自由度，均为 3×1 或 3×3。

---

## 单链 vs 整体：怎么选、怎么调

| 方式 | 含义 | 适用场景 | 主要函数 |
|------|------|----------|----------|
| **单链** | 每条腿/臂单独建一棵子树，只算该肢体的 M,C,G,τ,qdd | 只关心一条腿或一条臂；算单肢重力/惯性；不需与 Gazebo 严格一致 | `compute_e1_limb_dynamics`, `compute_e1_limb_forward_dynamics` |
| **整体** | 整机 18 DoF、固定基座，一次算全机 M,τ,qdd | 与 Gazebo 一致；需要肢体间耦合；对比实测用整机 FD 积分 | `compute_e1_dynamics`, `compute_e1_forward_dynamics_full` |

### 一、单链计算（按肢体）

每条肢体一条链，输入/输出维度 = 该肢体自由度（腿 6，臂 3）。

**逆动力学**（已知 q, qd, qdd → 求 τ，以及 M, C, G）：

```matlab
% 左腿：6 维
q_left   = rand(6, 1) * 0.2;
qd_left  = rand(6, 1) * 0.1;
qdd_left = rand(6, 1) * 0.05;
[tau_L, M_L, C_L, G_L] = compute_e1_limb_dynamics('left_leg', q_left, qd_left, qdd_left);
% tau_L, C_L, G_L 为 6×1，M_L 为 6×6

% 右腿、左臂、右臂同理，肢体名：'right_leg', 'left_arm', 'right_arm'
[tau_R, M_R, C_R, G_R] = compute_e1_limb_dynamics('right_leg', q_right, qd_right, qdd_right);
[M_arm, C_arm, G_arm]  = compute_e1_limb_dynamics('left_arm', q3, qd3);  % 仅要 M,C,G 可不传 qdd
```

**正动力学**（已知 q, qd, τ → 求 qdd）：

```matlab
qdd_left = compute_e1_limb_forward_dynamics('left_leg',  q_left,  qd_left,  tau_left);
qdd_right = compute_e1_limb_forward_dynamics('right_leg', q_right, qd_right, tau_right);
qdd_arm   = compute_e1_limb_forward_dynamics('right_arm', q3, qd3, tau3);
```

- 内部用 `get_e1_limb_robot(limb)` 得到该肢体的 subtree，再调用 Toolbox 的 `inverseDynamics` / `forwardDynamics`。
- 肢体间无耦合；若 E1 实际存在腿间耦合，与整机/Gazebo 结果会不同。

---

### 二、整体计算（整机 18 DoF）

整机、固定基座，关节顺序（与 URDF 一致）：**左臂 1–3，右臂 4–6，左腿 7–12，右腿 13–18**。

**逆动力学**（已知 q_18, qd_18, qdd_18 → 求 τ_18，以及 M, C, G）：

```matlab
q_full   = rand(18, 1) * 0.2;
qd_full  = rand(18, 1) * 0.1;
qdd_full = rand(18, 1) * 0.05;
[tau_full, M_full, C_full, G_full] = compute_e1_dynamics(q_full, qd_full, qdd_full);
% tau_full, C_full, G_full 为 18×1，M_full 为 18×18
```

**正动力学**（已知 q_18, qd_18, τ_18 → 求 qdd_18）：

```matlab
qdd_full = compute_e1_forward_dynamics_full(q_full, qd_full, tau_full);
% 若只关心腿：qdd_leg = qdd_full(7:18)；（左腿 7:12，右腿 13:18）
```

- 整机模型：`[robot, n, leg_idx] = get_e1_full_robot();`（Gravity=9.81，与 Gazebo 一致）。
- 与 Gazebo 固定基座、整机动力学一致；实测对比脚本 `compare_forward_dynamics_vs_measured.m` 即用整机 FD 积分。

---

### 三、整机由哪些单链组合？单链对应整机哪一块？

整机 18 维由四条单链组成，**E1.urdf 中顺序为左臂→右臂→左腿→右腿**，即左臂 1–3、右臂 4–6、左腿 7–12、右腿 13–18。因此：

- **不要假定**整机 1:6 就是左腿；应以**实际检测**为准。
- 使用 **`get_e1_full_robot_limb_indices()`**（在 `RobotModel/` 下）在零位用质量矩阵块匹配，得到各肢体在整机中的实际索引：
  - `[idx_ll, idx_rl, idx_la, idx_ra] = get_e1_full_robot_limb_indices()`
  - 左腿单链的 M、G、C 应与整机的 **M(idx_ll, idx_ll)**、**G(idx_ll)**、**C(idx_ll)** 对比；右腿、左臂、右臂同理。

| 单链 | 整机中对应块（需用检测结果） | 单链函数 |
|------|------------------------------|----------|
| 左腿 | M(idx_ll, idx_ll)、G(idx_ll)、C(idx_ll) | `get_e1_limb_robot('left_leg')`  |
| 右腿 | M(idx_rl, idx_rl)、G(idx_rl)、C(idx_rl) | `get_e1_limb_robot('right_leg')` |
| 左臂 | M(idx_la, idx_la)、G(idx_la)、C(idx_la) | `get_e1_limb_robot('left_arm')`  |
| 右臂 | M(idx_ra, idx_ra)、G(idx_ra)、C(idx_ra) | `get_e1_limb_robot('right_arm')` |

示例脚本：`compute_M_C_G_demo.m`（调用 `get_e1_full_robot_limb_indices()` 得到实际索引后，对比左腿单链与整机对应块）。

### 四、对比小结与推荐用法

| 项目 | 单链 | 整体 |
|------|------|------|
| 逆动力学 | `compute_e1_limb_dynamics(limb, q, qd, qdd)`，q/tau 等 = 该肢维度 | `compute_e1_dynamics(q_18, qd_18, qdd_18)`，18 维 |
| 正动力学 | `compute_e1_limb_forward_dynamics(limb, q, qd, tau)` | `compute_e1_forward_dynamics_full(q_18, qd_18, tau_18)` |
| 模型来源 | `get_e1_limb_robot(limb)` 子树 | `get_e1_full_robot()` 整机 |
| 与 Gazebo 一致 | 仅当 M 按肢块对角时一致 | 一致 |

**推荐**：新代码按肢体用 `compute_e1_limb_dynamics` / `compute_e1_limb_forward_dynamics`；仅需整机 18 维或与 Gazebo 对比时用 `compute_e1_dynamics` / `compute_e1_forward_dynamics_full`。肢体名：`'left_leg'` | `'right_leg'` | `'left_arm'` | `'right_arm'`。仅要 M、C、G 可不传 qdd：`[M, C, G] = compute_e1_limb_dynamics(limb, q, qd);`。

**整机正动力学（与 Gazebo 一致）**：臂无数据时置 0，`q_full = [zeros(6,1); q_leg];` 同理 qd、tau；`qdd_full = compute_e1_forward_dynamics_full(q_full, qd_full, tau_full);`，腿部分 `qdd_leg = qdd_full(7:18)`。

**实测轨迹对比**：用实测 q、qd、τ 做整机 FD 积分得 q_sim、qd_sim，与实测对比。积分用半隐式欧拉（与 Gazebo 一致）。脚本：`applications/PrincipleCalc/Body_GravityPara_Iden/compare_forward_dynamics_vs_measured.m`。

## 文件说明（本目录 E1 相关）

上级目录 **Legacy/** 下为通用机械臂（DH 模型）动力学/运动学；**E1（URDF）专用**在 **E1/**。完整列表见 [README.md](../README.md)。

| 文件 | 说明 |
|------|------|
| `compute_e1_limb_dynamics.m` | 按肢体逆动力学：tau, M, C, G |
| `compute_e1_limb_forward_dynamics.m` | 按肢体正动力学：qdd = FD(limb, q, qd, tau) |
| `compute_e1_dynamics.m` | 整机 18 维逆动力学（含耦合） |
| `compute_e1_forward_dynamics_full.m` | 整机 18 维正动力学（与 Gazebo 一致） |
| `check_e1_mass_matrix_coupling.m` | 检查腿/臂 M 块是否解耦 |
| `check_e1_gravity.m` | 重力自检 |
| `e1_example_joint_configs.m` | 示例关节角（零位、屈膝等） |
| `compute_M_C_G_demo.m` | 单链与整机 M/C/G 对比示例 |
| **RobotModel/** | `get_e1_full_robot`, `get_e1_limb_robot`, `get_e1_full_robot_limb_indices` |
| **文档** | 本目录 doc/：[E1_JOINT_ORDER.md](E1_JOINT_ORDER.md)、[E1_GAZEBO_JOINT_CONFIGS.md](E1_GAZEBO_JOINT_CONFIGS.md)、[E1_动力学一致性工程验证报告.md](E1_动力学一致性工程验证报告.md) |

## 动力学方程（单肢体）

对每个肢体成立：

```
τ = M(q)·qdd + C(q,qd) + G(q)
```

- **M(q)**：该肢体质量矩阵，腿 6×6、臂 3×3。  
- **C(q,qd)**：科氏/向心项，腿 6×1、臂 3×1。  
- **G(q)**：重力项，腿 6×1、臂 3×1。  
- **τ, q, qd, qdd**：该肢体关节力矩与位形/速度/加速度，维度与肢体一致。

## 全机 18 维（含耦合）与 Isaac Lab 对比

需全机耦合动力学时使用 `compute_e1_dynamics(q_18, qd_18, qdd_18)`（与按肢体分开算**不等价**：全机含肢体间耦合）。对比脚本见 `robot_algorithm/scripts/compare_with_isaac_lab.m`。

## 依赖与路径

- **E1 模型**：`noetix_description/urdf/E1.urdf`
- **MATLAB**：Robotics System Toolbox（`inverseDynamics`, `massMatrix`, `forwardDynamics`）
- 调用前将 `robot_algorithm/Dynamics/E1` 与 `robot_algorithm/RobotModel` 加入路径，或从项目根运行 `addpaths`

## 重力加速度怎么设

- **含义**：`robot.Gravity` 为**世界系**下的重力加速度向量（单位 m/s²）。整机与单链（左/右腿、臂）当前均设为 `[0; 0; -9.81]`。
- **悬空/固定**：机器人在空中被固定或站立时，世界系重力均为 `[0; 0; -9.81]`，不会变为零。
- **单链（左腿/右腿/臂）**：E1 各关节的 `<origin rpy="0 0 0">` 表示**零位（q=0）时**子 link 与父 link 同向；`<axis xyz="...">` 只表示运动时绕哪根轴转，不改变零位朝向。因此零位下所有链根与 world 同向，重力在链基座系下仍为 `[0; 0; -9.81]`。**质量矩阵 M** 的计算会用到每个关节的 axis（决定雅可比列方向），故轴不同 M 不同；rpy=0 只影响零位时重力方向一致，不影响 M/C 的结构。若某链根相对 world 有固定安装角，需在该链基座系下设对应重力方向。
- **约定**：世界系 **Z 轴向上**时，重力向下即为 `[0; 0; -9.81]`，与 ROS/Gazebo 一致。
- **如何自检**：运行 **`check_e1_gravity`**（幅值/方向检查，并从 URDF 汇总总质量）。若零位下 G 很小、担心模型有误，可将 `robot.Gravity` 临时改为 `[0; 9.81; 0]`（相当于机器人“侧放”，重力沿 Y）；此时零位 G 应在 M·g·0.1m 量级，可验证惯性参数与重力计算正确。

## 注意事项

1. **关节顺序**：与 [E1_JOINT_ORDER.md](E1_JOINT_ORDER.md) 一致；肢体内顺序与 URDF 该链一致。  
2. **单位**：角度 rad，角速度 rad/s，角加速度 rad/s²，力矩 N·m。  
3. **重力**：默认 `[0; 0; -9.81]`（世界系 Z 向上）；自检用 `check_e1_gravity`。  
4. **新代码**：一律使用 `compute_e1_limb_dynamics` / `compute_e1_limb_forward_dynamics`；仅在需要 18 维输出时使用 `compute_e1_dynamics`。
