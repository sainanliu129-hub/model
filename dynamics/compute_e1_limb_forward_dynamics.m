function qdd = compute_e1_limb_forward_dynamics(limb, q, qd, tau)
% compute_e1_limb_forward_dynamics  按肢体单独计算 E1 正动力学
%
% 给定当前肢体 q, qd 与关节力矩 tau，求加速度 qdd。维度均为该肢体自由度（腿 6，臂 3）。
%
% 用法:
%   qdd = compute_e1_limb_forward_dynamics(limb, q, qd, tau)
%
% 输入:
%   limb - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   q    - 关节位置 (rad)，列向量
%   qd   - 关节速度 (rad/s)
%   tau  - 关节力矩 (N·m)
%
% 输出:
%   qdd - 关节加速度 (rad/s^2)，列向量，满足 M*qdd = tau - C - G
%
% 依赖: get_e1_limb_robot, Robotics System Toolbox (forwardDynamics)

[robot_limb, n] = get_e1_limb_robot(limb);

if isrow(q),   q = q';   end
if isrow(qd),  qd = qd'; end
if isrow(tau), tau = tau'; end
if length(q) ~= n || length(qd) ~= n || length(tau) ~= n
    error('compute_e1_limb_forward_dynamics: 肢体 %s 自由度为 %d，输入长度须为 %d', limb, n, n);
end

qdd = forwardDynamics(robot_limb, q', qd', tau');
qdd = qdd';
% 若 Toolbox 返回 NaN/Inf（如惯性参数或位形导致质量矩阵病态），置为 0 避免积分发散
if any(~isfinite(qdd(:)))
    qdd(~isfinite(qdd)) = 0;
end
end
