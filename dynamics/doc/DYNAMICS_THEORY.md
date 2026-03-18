# 机器人动力学/逆动力学计算原理

> **E1 动力学接口与用法**见同目录 [E1_DYNAMICS_README.md](E1_DYNAMICS_README.md)。本文为通用理论说明。

## 1. 基本概念

### 1.1 动力学（Forward Dynamics）

**定义**：给定关节力矩，计算关节加速度

**输入**：
- 关节位置 `q` (N×1)
- 关节速度 `qd` (N×1)
- 关节力矩 `τ` (N×1)

**输出**：
- 关节加速度 `qdd` (N×1)

**问题**：已知 `q, qd, τ`，求 `qdd`

### 1.2 逆动力学（Inverse Dynamics）

**定义**：给定关节运动（位置、速度、加速度），计算所需的关节力矩

**输入**：
- 关节位置 `q` (N×1)
- 关节速度 `qd` (N×1)
- 关节加速度 `qdd` (N×1)

**输出**：
- 关节力矩 `τ` (N×1)

**问题**：已知 `q, qd, qdd`，求 `τ`

### 1.3 两者的关系

```
逆动力学：τ = f(q, qd, qdd)  →  已知运动，求力矩
正动力学：qdd = f(q, qd, τ)  →  已知力矩，求加速度
```

## 2. 动力学方程（拉格朗日方程）

### 2.1 标准形式

机器人动力学方程的标准形式为：

```
M(q) × qdd + C(q, qd) × qd + G(q) = τ
```

其中：
- **M(q)**：质量矩阵（惯性矩阵），N×N，对称正定
- **C(q, qd)**：科里奥利力和向心力矩阵，N×N
- **G(q)**：重力项，N×1
- **τ**：关节力矩，N×1
- **q, qd, qdd**：关节位置、速度、加速度，N×1

### 2.2 各项的物理意义

#### 质量矩阵 M(q)

- **物理意义**：描述系统的惯性特性
- **特点**：
  - 对称矩阵：`M = M^T`
  - 正定矩阵：`x^T M x > 0`（对所有非零向量 x）
  - 依赖于关节位置 `q`
- **对角线元素**：各关节的等效惯性
- **非对角线元素**：关节间的惯性耦合

#### 科里奥利力和向心力项 C(q, qd) × qd

- **物理意义**：
  - **科里奥利力**：由于坐标系旋转产生的惯性力
  - **向心力**：由于圆周运动产生的惯性力
- **特点**：
  - 依赖于关节位置 `q` 和速度 `qd`
  - 当速度为零时，该项为零
- **表达式**：`C(q, qd) × qd` 可以写成 `C(q, qd)` 矩阵乘以速度向量

#### 重力项 G(q)

- **物理意义**：重力对各关节产生的力矩
- **特点**：
  - 只依赖于关节位置 `q`
  - 当速度为零、加速度为零时，`τ = G(q)`
- **计算**：`G(q) = τ(q, 0, 0)`（零速度、零加速度时的力矩）

### 2.3 动力学方程的分解

将动力学方程重新组织：

```
τ = M(q) × qdd + C(q, qd) × qd + G(q)
```

可以分解为：

```
τ = M(q) × qdd + [C(q, qd) × qd + G(q)]
  = M(q) × qdd + H(q, qd)
```

其中 `H(q, qd) = C(q, qd) × qd + G(q)` 是科里奥利力、向心力和重力的组合项。

## 3. 逆动力学计算（Inverse Dynamics）

### 3.1 计算方法

**直接计算**：给定 `q, qd, qdd`，直接计算 `τ`

```
τ = M(q) × qdd + C(q, qd) × qd + G(q)
```

### 3.2 算法：递归牛顿-欧拉算法（Recursive Newton-Euler）

这是计算逆动力学最常用的算法，分为两个阶段：

#### 阶段 1：前向传播（Forward Pass）

从基座到末端，计算每个连杆的速度和加速度：

```matlab
% 初始化基座状态
w0 = [0; 0; 0];      % 基座角速度
wd0 = [0; 0; 0];     % 基座角加速度
v0 = [0; 0; 0];      % 基座线速度
vd0 = -g;            % 基座线加速度（包含重力）

% 对每个关节 i = 1 到 N
for i = 1:N
    % 计算关节 i 的角速度和角加速度（在关节坐标系中）
    w{i} = R{i}' * w{i-1} + qd(i) * e;
    wd{i} = R{i}' * wd{i-1} + qdd(i) * e + cross(R{i}'*w{i-1}, qd(i)*e);
    
    % 计算关节 i 的线速度和线加速度（在关节坐标系中）
    v{i} = R{i}' * (v{i-1} + cross(w{i-1}, p{i}));
    vd{i} = R{i}' * (vd{i-1} + cross(w{i-1}, cross(w{i-1}, p{i})) + cross(wd{i-1}, p{i}));
    
    % 计算质心速度和加速度
    vc{i} = v{i} + cross(w{i}, c{i});
    vcd{i} = vd{i} + cross(wd{i}, c{i}) + cross(w{i}, cross(w{i}, c{i}));
    
    % 计算作用在质心处的力和力矩
    F{i} = m(i) * vcd{i};
    T{i} = Ic{i} * wd{i} + cross(w{i}, Ic{i} * w{i});
end
```

#### 阶段 2：后向传播（Backward Pass）

从末端到基座，计算每个关节的力矩：

```matlab
% 初始化末端力/力矩
f{N+1} = [0; 0; 0];
t{N+1} = [0; 0; 0];

% 对每个关节 i = N 到 1
for i = N:-1:1
    % 计算作用在关节 i 上的力和力矩
    f{i} = R{i+1} * f{i+1} + F{i};
    t{i} = T{i} + R{i+1} * t{i+1} + cross(c{i}, F{i}) + cross(p{i+1}, R{i+1} * f{i+1});
    
    % 计算关节力矩（投影到关节轴上）
    tau(i) = t{i}' * e + I_motor(i) * qdd(i);
end
```

### 3.3 MATLAB 实现

```matlab
function tau = inverse_dynamics(robot, q, qd, qdd)
    % 使用 MATLAB Robotics System Toolbox
    tau = inverseDynamics(robot, q', qd', qdd');
    tau = tau';
end
```

## 4. 正动力学计算（Forward Dynamics）

### 4.1 计算方法

**直接计算**：给定 `q, qd, τ`，计算 `qdd`

从动力学方程：
```
M(q) × qdd + C(q, qd) × qd + G(q) = τ
```

解出加速度：
```
qdd = M(q)^(-1) × [τ - C(q, qd) × qd - G(q)]
```

### 4.2 算法步骤

1. **计算质量矩阵 M(q)**
   ```matlab
   M = massMatrix(robot, q');
   ```

2. **计算重力项 G(q)**
   ```matlab
   G = inverseDynamics(robot, q', zeros(N,1), zeros(N,1));
   G = G';
   ```

3. **计算科里奥利力和向心力项 C(q, qd) × qd**
   ```matlab
   % 方法1：直接计算
   tau_zero_acc = inverseDynamics(robot, q', qd', zeros(N,1));
   C_times_qd = tau_zero_acc' - G;
   
   % 方法2：使用质量矩阵的导数（更复杂）
   ```

4. **计算加速度**
   ```matlab
   qdd = M \ (tau - C_times_qd - G);
   ```

### 4.3 MATLAB 实现

```matlab
function qdd = forward_dynamics(robot, q, qd, tau)
    % 计算质量矩阵
    M = massMatrix(robot, q');
    
    % 计算重力项
    G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qd')));
    G = G';
    
    % 计算科里奥利力和向心力项
    tau_zero_acc = inverseDynamics(robot, q', qd', zeros(size(qd')));
    C_times_qd = tau_zero_acc' - G;
    
    % 计算加速度
    qdd = M \ (tau - C_times_qd - G);
    qdd = qdd(:);  % 确保是列向量
end
```

## 5. 质量矩阵的计算

### 5.1 理论公式

质量矩阵可以通过雅可比矩阵计算：

```
M(q) = Σ [J_i^T × M_i × J_i]
```

其中：
- `J_i`：第 i 个连杆的雅可比矩阵（6×N）
- `M_i`：第 i 个连杆的空间惯性矩阵（6×6）
- 求和是对所有连杆

### 5.2 空间惯性矩阵

对于第 i 个连杆，空间惯性矩阵为：

```
M_i = [I_i + m_i × S(c_i)^T × S(c_i)    m_i × S(c_i)^T    ]
      [m_i × S(c_i)                      m_i × I_3        ]
```

其中：
- `I_i`：连杆在质心处的惯性张量（3×3）
- `m_i`：连杆质量
- `c_i`：质心位置（在关节坐标系中）
- `S(c_i)`：质心位置的反对称矩阵

### 5.3 MATLAB 实现

```matlab
function M = compute_mass_matrix(robot, q)
    % 使用 MATLAB Robotics System Toolbox
    M = massMatrix(robot, q');
end
```

## 6. 科里奥利力和向心力项的计算

### 6.1 理论公式

科里奥利力和向心力矩阵可以通过质量矩阵的导数计算：

```
C(q, qd) = (1/2) × [dM/dq × qd + dM^T/dq × qd - dM/dq^T × qd]
```

但这种方法计算复杂，通常使用以下方法：

### 6.2 计算方法

**方法1：通过逆动力学计算**

```
C(q, qd) × qd = τ(q, qd, 0) - G(q)
```

即：给定位置和速度，零加速度时的力矩减去重力项。

**方法2：使用 Christoffel 符号**

```
C_ij = (1/2) × Σ [dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i] × qd_k
```

### 6.3 MATLAB 实现

```matlab
% 计算 C × qd
tau_zero_acc = inverseDynamics(robot, q', qd', zeros(size(qd')));
G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qd')));
C_times_qd = tau_zero_acc' - G';
```

## 7. 重力项的计算

### 7.1 理论公式

重力项是零速度、零加速度时的力矩：

```
G(q) = τ(q, 0, 0)
```

### 7.2 物理意义

重力项表示重力对各关节产生的力矩，可以通过虚功原理计算：

```
G_i(q) = -∂V/∂q_i
```

其中 `V` 是系统的势能。

### 7.3 MATLAB 实现

```matlab
% 计算重力项
G = inverseDynamics(robot, q', zeros(N,1), zeros(N,1));
G = G';
```

## 8. 完整计算流程

### 8.1 逆动力学流程

```
输入: q, qd, qdd
  ↓
1. 前向传播：计算各连杆速度、加速度
  ↓
2. 计算各连杆的惯性力/力矩
  ↓
3. 后向传播：计算各关节力矩
  ↓
输出: τ
```

### 8.2 正动力学流程

```
输入: q, qd, τ
  ↓
1. 计算质量矩阵 M(q)
  ↓
2. 计算重力项 G(q)
  ↓
3. 计算科里奥利力和向心力项 C(q,qd)×qd
  ↓
4. 求解线性方程组：M(q)×qdd = τ - C×qd - G
  ↓
输出: qdd
```

## 9. 数值计算注意事项

### 9.1 质量矩阵求逆

质量矩阵 `M(q)` 是正定矩阵，因此：
- 总是可逆
- 可以使用 `M \ b` 或 `inv(M) * b` 求解
- 推荐使用 `\` 运算符（更稳定、更快）

### 9.2 数值稳定性

- **质量矩阵**：通常条件数较大，需要小心处理
- **科里奥利力项**：当速度很小时，该项可以忽略
- **重力项**：在零位附近，重力项通常较小

### 9.3 计算复杂度

- **逆动力学**：O(N)，其中 N 是关节数
- **正动力学**：O(N²) 或 O(N³)，取决于质量矩阵求逆的方法
- **质量矩阵计算**：O(N²)

## 10. 代码示例

### 10.1 完整的逆动力学计算

```matlab
function tau = compute_inverse_dynamics(robot, q, qd, qdd)
    % 输入：q, qd, qdd (列向量)
    % 输出：tau (列向量)
    
    % 使用 MATLAB Robotics System Toolbox
    tau = inverseDynamics(robot, q', qd', qdd');
    tau = tau';
end
```

### 10.2 完整的正动力学计算

```matlab
function qdd = compute_forward_dynamics(robot, q, qd, tau)
    % 输入：q, qd, tau (列向量)
    % 输出：qdd (列向量)
    
    % 1. 计算质量矩阵
    M = massMatrix(robot, q');
    
    % 2. 计算重力项
    G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qd')));
    G = G';
    
    % 3. 计算科里奥利力和向心力项
    tau_zero_acc = inverseDynamics(robot, q', qd', zeros(size(qd')));
    C_times_qd = tau_zero_acc' - G;
    
    % 4. 计算加速度
    qdd = M \ (tau - C_times_qd - G);
    qdd = qdd(:);
end
```

### 10.3 分解各项的计算

```matlab
function [M, C_times_qd, G] = compute_dynamics_terms(robot, q, qd)
    % 计算动力学各项
    
    % 质量矩阵
    M = massMatrix(robot, q');
    
    % 重力项
    G = inverseDynamics(robot, q', zeros(size(qd')), zeros(size(qd')));
    G = G';
    
    % 科里奥利力和向心力项
    tau_zero_acc = inverseDynamics(robot, q', qd', zeros(size(qd')));
    C_times_qd = tau_zero_acc' - G;
end
```

## 11. 总结

### 11.1 关键公式

**动力学方程**：
```
M(q) × qdd + C(q, qd) × qd + G(q) = τ
```

**逆动力学**：
```
τ = M(q) × qdd + C(q, qd) × qd + G(q)
```

**正动力学**：
```
qdd = M(q)^(-1) × [τ - C(q, qd) × qd - G(q)]
```

### 11.2 算法选择

- **逆动力学**：使用递归牛顿-欧拉算法（O(N)）
- **正动力学**：使用质量矩阵求逆（O(N²) 或 O(N³)）
- **质量矩阵**：通过雅可比矩阵计算（O(N²)）

### 11.3 实际应用

- **控制器设计**：需要逆动力学计算期望力矩
- **仿真**：需要正动力学计算加速度并积分
- **参数辨识**：需要质量矩阵和重力项

## 参考

- 《机器人学导论》（Introduction to Robotics）- John J. Craig
- 《机器人动力学与控制》（Robot Dynamics and Control）- Mark W. Spong
- MATLAB Robotics System Toolbox 文档
