function [t_seg, q_seg] = movej_segment(q_start, q_end, v_max_used, accel_ratio, Ts)
% movej_segment  生成单段MOVEJ轨迹（5次多项式，位置速度加速度连续）
%
% 用于段间平滑过渡，保证无突变。
% 边界条件：起止位置给定，起止速度、加速度为0。
%
% 输入:
%   q_start, q_end - 起点、终点位置 (rad)
%   v_max_used     - 用于规划的最大速度 (rad/s)
%   accel_ratio    - 加速度比例，a_max = v_max_used * accel_ratio
%   Ts             - 采样周期 (s)
% 输出:
%   t_seg - 相对时间向量 (从0开始)
%   q_seg - 位置向量

dist = abs(q_end - q_start);
if dist < 1e-6
    t_seg = [0; Ts];
    q_seg = [q_start; q_start];
    return;
end

a_max = v_max_used * accel_ratio;
T_from_vel = 15/8 * dist / v_max_used;
T_from_accel = sqrt(6 * dist / a_max);
T = max(T_from_vel, T_from_accel);
T = max(T, 0.5);

n_points = max(round(T / Ts), 10);
s = linspace(0, 1, n_points)';
delta_q = q_end - q_start;
a0 = q_start;
a1 = 0;
a2 = 0;
a3 = 10 * delta_q;
a4 = -15 * delta_q;
a5 = 6 * delta_q;

t_seg = s * T;
q_seg = a0 + a1*s + a2*s.^2 + a3*s.^3 + a4*s.^4 + a5*s.^5;
q_seg(1) = q_start;
q_seg(end) = q_end;
end
