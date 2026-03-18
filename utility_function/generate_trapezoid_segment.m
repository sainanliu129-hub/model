function [t, q, qd, qdd] = generate_trapezoid_segment(q_start, q_end, v_target, a_max, dt)
% generate_trapezoid_segment  单关节标准 T 型速度轨迹（加速-匀速-减速）
%
% 轨迹特性：
%   - 起点 q_start、终点 q_end
%   - 初始/终止速度为 0
%   - 先用恒定加速度 a_max 加速到目标速度 v_target
%   - 再以恒定 v_target 匀速一段
%   - 再用 -a_max 减速到 0
%   - 若距离太短，无法达到 v_target，则自动退化为“三角速度”（仅加速+减速）
%
% 输入：
%   q_start  - 起始关节角 (rad)
%   q_end    - 终止关节角 (rad)
%   v_target - 期望匀速速度大小 (rad/s)，取正数即可，方向由 q_end-q_start 决定
%   a_max    - 最大加速度大小 (rad/s^2)，取正数
%   dt       - 采样时间 (s)
%
% 输出：
%   t   - 时间列向量，从 0 开始
%   q   - 与 t 对应的关节角轨迹（规划值）
%   qd  - 与 t 对应的角速度（规划值，非微分）
%   qdd - 与 t 对应的角加速度（规划值，非微分）

if dt <= 0
    error('dt 必须为正数');
end

L = q_end - q_start;
dist = abs(L);
if dist < 1e-9
    t = (0:1)' * dt;
    q = q_start * ones(2, 1);
    qd = zeros(2, 1);
    qdd = zeros(2, 1);
    return;
end

dir = sign(L);
v_abs = abs(v_target);
a_abs = abs(a_max);

if v_abs <= 0 || a_abs <= 0
    % 无有效速度/加速度信息，退化为线性插值
    N = max(2, ceil(dist / dt));
    t = (0:N-1)' * dt;
    q = linspace(q_start, q_end, N).';
    qd = (q_end - q_start) / max(t(end), dt) * ones(N, 1);
    qdd = zeros(N, 1);
    return;
end

% 加速段所需距离（恒定加速度）
s_acc = v_abs^2 / (2 * a_abs);

if 2 * s_acc <= dist
    % 距离足够：标准 T 型（有匀速段）
    s_const = dist - 2 * s_acc;
    t_acc   = v_abs / a_abs;
    t_const = s_const / v_abs;

    n_acc   = max(2, ceil(t_acc / dt));
    n_const = max(0, ceil(t_const / dt));

    t1 = (0:n_acc-1).' * dt;
    t2 = (1:n_const).' * dt + t1(end);
    t3 = (1:n_acc).'   * dt + (isempty(t2) * t1(end) + (~isempty(t2)) * (t2(end)));

    % 加速：q = q_start + 0.5*a*t^2*dir, qd = a*t*dir, qdd = a*dir
    q1   = q_start + dir * 0.5 * a_abs * t1.^2;
    qd1  = dir * a_abs * t1;
    qdd1 = dir * a_abs * ones(n_acc, 1);

    % 匀速段：qd = v*dir, qdd = 0
    if n_const > 0
        q2   = q1(end) + dir * v_abs * (t2 - t2(1));
        qd2  = dir * v_abs * ones(n_const, 1);
        qdd2 = zeros(n_const, 1);
    else
        q2 = []; qd2 = []; qdd2 = [];
    end

    % 减速：t_dec 从 0 增加，q = q_end - 0.5*a*t_dec^2*dir, qd = v*dir - a*t_dec*dir, qdd = -a*dir
    t_dec = t3 - t3(1);
    q3   = q_end - dir * 0.5 * a_abs * t_dec.^2;
    qd3  = dir * (v_abs - a_abs * t_dec);
    qdd3 = -dir * a_abs * ones(n_acc, 1);

    t   = [t1; t2; t3];
    q   = [q1; q2; q3];
    qd  = [qd1; qd2; qd3];
    qdd = [qdd1; qdd2; qdd3];
else
    % 距离不足：三角速度（达不到 v_target）
    t_acc = sqrt(dist / a_abs);
    v_peak = a_abs * t_acc;
    n_acc = max(2, ceil(t_acc / dt));

    t1 = (0:n_acc-1).' * dt;
    t2 = (1:n_acc).'   * dt + t1(end);

    q1   = q_start + dir * 0.5 * a_abs * t1.^2;
    qd1  = dir * a_abs * t1;
    qdd1 = dir * a_abs * ones(n_acc, 1);

    t_dec = t2 - t2(1);
    q2   = q_end - dir * 0.5 * a_abs * t_dec.^2;
    qd2  = dir * (v_peak - a_abs * t_dec);
    qdd2 = -dir * a_abs * ones(n_acc, 1);

    t   = [t1; t2];
    q   = [q1; q2];
    qd  = [qd1; qd2];
    qdd = [qdd1; qdd2];
end

% 数值修正端点
q(1)   = q_start;
q(end) = q_end;
qd(1)  = 0;
qd(end)= 0;

end

