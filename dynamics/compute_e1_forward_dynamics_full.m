function qdd = compute_e1_forward_dynamics_full(q, qd, tau)
% compute_e1_forward_dynamics_full   E1 整机正动力学（与 Gazebo 一致）
%
% 使用整机、固定基座模型计算正动力学，与 Gazebo 仿真一致（无子树截断）。
% 输入/输出顺序：左臂 1–3，右臂 4–6，左腿 7–12，右腿 13–18（与 URDF 一致）。
%
% 用法:
%   qdd = compute_e1_forward_dynamics_full(q, qd, tau)
%
% 输入:
%   q   - 18×1 或 1×18 关节位置 (rad)
%   qd  - 18×1 或 1×18 关节速度 (rad/s)
%   tau - 18×1 或 1×18 关节力矩 (N·m)
%
% 输出:
%   qdd - 18×1 关节加速度 (rad/s^2)，M*qdd = tau - C - G
%
% 依赖: get_e1_full_robot, Robotics System Toolbox (forwardDynamics)

[robot, n] = get_e1_full_robot();
if isrow(q),   q = q';   end
if isrow(qd),  qd = qd'; end
if isrow(tau), tau = tau'; end
if length(q) ~= n || length(qd) ~= n || length(tau) ~= n
    error('compute_e1_forward_dynamics_full: 期望长度 %d，输入为 %d', n, length(q));
end

qdd = forwardDynamics(robot, q', qd', tau');
qdd = qdd(:);
if any(~isfinite(qdd))
    qdd(~isfinite(qdd)) = 0;
end
end
