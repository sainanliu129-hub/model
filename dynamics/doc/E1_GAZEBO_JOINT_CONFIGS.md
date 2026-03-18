# E1 示例关节角与 Gazebo 中仿真对照

> 动力学计算主说明见同目录 [E1_DYNAMICS_README.md](E1_DYNAMICS_README.md)。本文仅说明示例关节角及 Gazebo 对照。

## 一、MATLAB 中几组示例关节角

在 MATLAB 中运行：

```matlab
configs = e1_example_joint_configs();
```

得到的 `configs` 包含以下配置（均为 18×1，单位 rad），顺序：**左臂 1–3，右臂 4–6，左腿 7–12，右腿 13–18**。

| 配置名 | 说明 |
|--------|------|
| `q_zero` | 全零位 |
| `q_left_knee_bent` | 左腿膝弯（左 knee≈0.5 rad，hip_pitch≈-0.25 rad） |
| `q_right_knee_bent` | 右腿膝弯（对称） |
| `q_arms_raised` | 双臂前抬（左/右 shoulder_pitch≈0.4 rad） |
| `q_random_small` | 各关节小随机偏移（测试用） |
| `q_squat` | 双腿微蹲（两膝弯） |

示例：取零位和左膝弯的数值

```matlab
configs = e1_example_joint_configs();
q0 = configs.q_zero;
q1 = configs.q_left_knee_bent;
[~,~,~,G0] = compute_e1_dynamics(q0, zeros(18,1), zeros(18,1));
[~,~,~,G1] = compute_e1_dynamics(q1, zeros(18,1), zeros(18,1));
fprintf('零位 max|G| = %.2f, 左膝弯 max|G| = %.2f\n', max(abs(G0)), max(abs(G1)));
```

---

## 二、整机 18 维与 Gazebo 关节名对应

Gazebo/ROS 中关节名与 MATLAB 下标一一对应（与 URDF 一致）：

| 下标 | 关节名 (ROS/Gazebo) |
|------|----------------------|
| 1–3  | `arm_l1_joint`, `arm_l2_joint`, `arm_l3_joint` |
| 4–6  | `arm_r1_joint`, `arm_r2_joint`, `arm_r3_joint` |
| 7–12 | `leg_l1_joint`, …, `leg_l6_joint` |
| 13–18| `leg_r1_joint`, …, `leg_r6_joint` |

即：

```
arm_l1_joint, arm_l2_joint, arm_l3_joint,
arm_r1_joint, arm_r2_joint, arm_r3_joint,
leg_l1_joint, leg_l2_joint, leg_l3_joint, leg_l4_joint, leg_l5_joint, leg_l6_joint,
leg_r1_joint, leg_r2_joint, leg_r3_joint, leg_r4_joint, leg_r5_joint, leg_r6_joint
```

---

## 三、在 Gazebo 里怎么“给这几个关节角”并仿真

E1 的 URDF 使用 `liblegged_hw_sim.so`（LeggedHWSim），控制接口以 **EffortJointInterface**（力矩控制）为主。因此“给关节角”通常不是直接发角度，而是**通过控制器发力矩或目标位置**，由仿真步进得到对应姿态。

### 方式 1：用 ROS 控制接口发目标位置/力矩（推荐）

1. **启动 Gazebo 与 E1**  
   用你们项目里已有的 launch（例如 `bringup_leg.launch` 或包含 `legged_gazebo` 的 launch），确保加载了 E1 的 URDF 和 `gazebo_ros_control` / `legged_hw_sim`。

2. **确认控制器与话题**  
   - 查看当前控制器：`rosservice call /controller_manager/list_controllers`  
   - 或：`rostopic list | grep joint`  
   常见情况会看到每个关节一个 effort 控制器，例如：  
   `/leg_l1_joint_effort_controller/command` 等。

3. **发“目标关节角”的两种常见做法**  
   - **若有 position 接口**：对每个关节的 position 控制器发目标位置（rad），即可在 Gazebo 里“给这几个关节角”。  
   - **若只有 effort 接口**：在外部写一个简单 PD 节点：  
     - 订阅当前关节状态（如 `/joint_states`），  
     - 用目标角与当前角算误差，再算力矩：`tau = Kp*(q_des - q) - Kd*qd`，  
     - 把 `tau` 发到对应 `/xxx_joint_effort_controller/command`。  
     这样在 Gazebo 里就会稳定到你要的关节角并持续仿真。

4. **用本仓库给的几组角做“目标”**  
   - 在 MATLAB 里：`configs = e1_example_joint_configs(); q_des = configs.q_left_knee_bent;`  
   - 按上表把 `q_des(1)`～`q_des(18)` 依次对应到 `arm_l1_joint`～`leg_r6_joint`。  
   - 在 ROS 里把这些值作为目标位置发给 position 控制器，或作为 PD 的 `q_des` 再发力矩到 effort 控制器。  
   这样就是在 Gazebo 里“用这几组关节角”做仿真。

### 方式 2：用 joint_state_publisher / GUI（仅改显示或简单测试）

- 若只是想让模型在 RViz/Gazebo 里**摆出**这些角度，可以用：  
  `joint_state_publisher_gui` 或 `joint_state_publisher`（带 yaml 配置），发布 `/joint_states`。  
- 注意：很多 Gazebo 仿真**不用** `/joint_states` 作为控制输入，只用于显示；实际动力学仍由 effort/position 控制器驱动。所以若要“仿真在这几个关节角下动力学是否正确”，仍需用方式 1 通过控制器把机器人控制到这些角度。

### 方式 3：一次性地把关节角发给 Gazebo（若支持）

- 若你们的 `legged_hw_sim` 或自定义插件支持“设置初始关节位置”，可在 spawn 或 reset 时传入上述 18 个关节角（顺序同上）。  
- 否则，一般是通过方式 1 在仿真跑起来后，用目标位置或 PD 把机器人驱动到这几组角。

---

## 四、和 MATLAB 动力学对比（重力矩 G）

1. **在 Gazebo 中**：用方式 1 把机器人控制到某一组关节角（如 `q_left_knee_bent`），稳定后读取 `/joint_states` 得到实际 `q`（以及 `qd` 若存在）。若控制器有力矩反馈话题，也可记下该时刻的关节力矩。  
2. **在 MATLAB 中**：  
   ```matlab
   q_gazebo = [ ... ]; % 从 /joint_states 按 arm_l1..leg_r6 顺序排成 18×1
   [~,~,~,G] = compute_e1_dynamics(q_gazebo, zeros(18,1), zeros(18,1));
   ```  
   比较 Gazebo 显示的力矩与 MATLAB 的 `G`（静止时理论应为重力矩）。  
3. **关节顺序**：务必保证 Gazebo 里读到的关节顺序与上面“整机 18 维”一致（左臂 1–3、右臂 4–6、左腿 7–12、右腿 13–18），否则要对齐后再对比。

---

## 五、小结

- **示例关节角**：在 MATLAB 里用 `e1_example_joint_configs()` 得到多组 18×1 的 `q`（零位、左/右膝弯、抬臂、随机、微蹲等）。  
- **在 Gazebo 里仿真**：通过 ROS 的 position 或 effort 控制器，把上述 `q` 作为目标角（或 PD 目标）发给对应 18 个关节（关节名见上表），即可在 Gazebo 中复现这几组关节角并做动力学仿真。  
- **对比**：从 Gazebo 读 `/joint_states` 得到实际 `q`，在 MATLAB 用 `compute_e1_dynamics(q,0,0)` 算 G，与 Gazebo 力矩对比即可验证一致性。
