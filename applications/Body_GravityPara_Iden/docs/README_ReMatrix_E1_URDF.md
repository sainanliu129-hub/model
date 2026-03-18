# E1 单腿基于电流的动力学辨识矩阵（URDF）

## 目的

**仅单腿、基于关节电流（力矩）**的动力学参数辨识矩阵，tau = Y * theta。不依赖 DH 与 calMatrix_kinHomTrans，由 E1 URDF 的 rigidBodyTree + getTransform 得到运动学，再按与 ReMatrix_RobotDynPara 相同的 A/HT/Yii 公式计算 Y_Tauj。

- 不做底座力（Y_Fbase）回归；
- 仅支持 left_leg / right_leg，各 6 自由度。

## 接口

### 主函数：`ReMatrix_E1_limb_URDF.m`

```
Y = ReMatrix_E1_limb_URDF(limb, qfull, qdfull, qddfull, output_type, para_order)
```

| 参数 | 说明 |
|------|------|
| `limb` | `'left_leg'` \| `'right_leg'`（仅单腿） |
| `qfull` | N×6 关节位置 (rad) |
| `qdfull` | N×6 关节速度 (rad/s) |
| `qddfull` | N×6 关节加速度 (rad/s²) |
| `output_type` | 1：60 参数 Y_Tauj；2：66 参数（含转子惯量 Ia） |
| `para_order` | 1 或 2，参数顺序同 ReMatrix_RobotDynPara |

**输出**：观测矩阵 Y，(N×6)×60 或 (N×6)×66，满足 tau = Y * theta。

## 使用示例

```matlab
addpath('path/to/Body_GravityPara_Iden');
addpath('path/to/robot_model');

% 单点
q = zeros(1, 6); qd = zeros(1, 6); qdd = zeros(1, 6);
Y = ReMatrix_E1_limb_URDF('left_leg', q, qd, qdd, 1, 2);

% 轨迹（辨识或条件数）
Y = ReMatrix_E1_limb_URDF('left_leg', refPos, refVel, refAcc, 1, 2);
cond_Y = cond(Y' * Y);
```

## 与激励轨迹条件数优化结合

```matlab
cond_fun = @(refPos, refVel, refAcc) cond(ReMatrix_E1_limb_URDF('left_leg', refPos, refVel, refAcc, 1, 2));
[~, trajPara, ...] = generate_excitation_trajectory(config, 'optimize', true, 'cond_fun', cond_fun);
```

需保证 config.move_axis、refPos/refVel/refAcc 与单腿 6 关节对应。

## 模型与参数约定

- **无单独 base 体**：单腿模型没有“底座刚体”，直接是 6 个关节角与 6 个 link；`Bodies{1}`..`Bodies{6}` 即 6 个连杆，关节 i 在 link i-1 与 link i 之间（link 0 为 subtree 根坐标系）。
- **URDF 惯量**：URDF 中 `<inertial>` 的惯量由**质心**定义且与指定坐标系对齐；`importrobot` 导入时会将其转换到**连杆坐标系（body frame）**，即 `rigidBody.Inertia` 为**相对 body 原点**的惯量，`CenterOfMass` 为该 body 系下的质心位置。辨识矩阵 Y 与 θ 使用同一约定（m, m·com, I 均在 body 系）。
- **inverseDynamics 与 θ 顺序一致**：Toolbox 的 `inverseDynamics` 使用每体的 `Mass`、`CenterOfMass`、`Inertia`，顺序为 `Bodies{1}`..`Bodies{n}`；`Inertia` 为 `[Ixx Iyy Izz Iyz Ixz Ixy]`。`get_limb_theta_from_URDF` 按同一 body 顺序、同一 10 参顺序（para_order 1: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz）拼成 θ，故 τ_ref = inverseDynamics(...) 与 τ_Y = Y*θ 可直接对比验收。

## 实现要点与验收

为保证 Y 物理一致、辨识可信，实现中需注意：

- **基座名**：用 `robot_limb.BaseName` 作为 getTransform 的参考，不用 `Bodies{1}`。
- **相对变换**：递推与 HT 使用相邻相对变换 `Trel{i}=inv(T0{i-1})*T0{i}` 的 R、p，不用绝对 ^0T_i 的 R、p。
- **关节轴**：从 `Bodies{i}.Joint.JointAxis` 读取并单位化，不写死 [0;0;1]；**Yii 力矩投影**须用 `[0 0 0 z_r']`（沿关节轴 z_r），不可用固定 `[0 0 0 0 0 1]`。
- **预分配**：Y 按 `zeros(N*n, 10*n)` 或 `11*n` 预分配，避免逐点 `Y=[Y;...]`。

**验收（强烈建议）**：在 URDF 一致环境下，随机多组 (q,qd,qdd)，用 `inverseDynamics(robot,q,qd,qdd)` 得 τ_ref，用 Y(q,qd,qdd)*θ（θ 从 URDF 的 link 惯性按同一参数顺序拼出）得 τ_Y，检查 |τ_ref − τ_Y| 在数值误差量级（如 <1e-2 N·m）。若不符，说明 base/相对变换/轴/链序等仍有误。

- 脚本 **`test_ReMatrix_E1_limb_URDF.m`** 自动完成上述验收（多组随机点 + 零位）；依赖 **`get_limb_theta_from_URDF.m`** 从 rigidBodyTree 按 para_order 拼出 θ。

## 依赖

- `get_e1_limb_robot`（本仓库 `robot_model`）
- `noetix_description/urdf/E1.urdf`
- Robotics System Toolbox：`getTransform`、`homeConfiguration`
