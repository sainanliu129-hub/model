% check_moveJ_velocity  检查 moveJ 轨迹的实际最大速度是否达到期望值
%
% 问题：5次多项式轨迹使用近似公式 T ≈ 15/8 * dist / v_max 计算时间，
%       但实际最大速度可能小于期望的 v_max_used
%
% 本脚本验证并修复：计算实际速度曲线，确保最大速度达到期望值

clear; clc;

%% 测试参数
dist = 1.0;  % 运动距离 (rad)
v_max_desired = [1.0, 2.0, 3.0];  % 期望的最大速度 (rad/s)
accel_ratio = 0.5;
Ts = 0.002;

fprintf('===== 检查 moveJ 轨迹实际速度 =====\n');
fprintf('运动距离: %.2f rad\n', dist);
fprintf('加速度比例: %.2f\n', accel_ratio);
fprintf('采样周期: %.4f s\n', Ts);

for i = 1:length(v_max_desired)
    v_max_used = v_max_desired(i);
    fprintf('\n--- 期望最大速度: %.2f rad/s ---\n', v_max_used);
    
    % 使用当前方法计算时间
    a_max = v_max_used * accel_ratio;
    T_from_vel = 15/8 * dist / v_max_used;
    T_from_accel = sqrt(6 * dist / a_max);
    T = max(T_from_vel, T_from_accel);
    T = max(T, 0.5);
    
    fprintf('计算时间: T_from_vel=%.3f s, T_from_accel=%.3f s, T=%.3f s\n', ...
        T_from_vel, T_from_accel, T);
    
    % 生成5次多项式轨迹
    n_points = max(round(T / Ts), 10);
    s = linspace(0, 1, n_points)';
    delta_q = dist;
    a3 = 10 * delta_q;
    a4 = -15 * delta_q;
    a5 = 6 * delta_q;
    
    % 位置
    q = a3*s.^3 + a4*s.^4 + a5*s.^5;
    
    % 速度（对位置求导，注意归一化时间s到实际时间t的转换：t = s*T）
    % qd = dq/dt = dq/ds * ds/dt = dq/ds / T
    % dq/ds = 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
    qd_normalized = 3*a3*s.^2 + 4*a4*s.^3 + 5*a5*s.^4;
    qd = qd_normalized / T;  % 实际速度
    
    % 加速度
    qdd_normalized = 6*a3*s + 12*a4*s.^2 + 20*a5*s.^3;
    qdd = qdd_normalized / T^2;
    
    % 找到实际最大速度
    [v_max_actual, idx_max] = max(abs(qd));
    fprintf('实际最大速度: %.4f rad/s (出现在 s=%.3f, t=%.3f s)\n', ...
        v_max_actual, s(idx_max), s(idx_max)*T);
    fprintf('速度误差: %.2f%% (期望 %.2f, 实际 %.4f)\n', ...
        (v_max_used - v_max_actual) / v_max_used * 100, v_max_used, v_max_actual);
    
    % 如果误差超过5%，需要调整时间
    if abs(v_max_used - v_max_actual) / v_max_used > 0.05
        fprintf('⚠️  速度误差超过5%%，需要调整时间\n');
        % 根据实际速度公式反推时间
        % v_max_actual ≈ 15/8 * dist / T
        % 所以 T_corrected = 15/8 * dist / v_max_used（但这是近似）
        % 更准确的方法：迭代调整T，使实际最大速度接近v_max_used
        T_corrected = T * v_max_used / v_max_actual;
        fprintf('建议调整时间: T=%.3f -> T_corrected=%.3f s\n', T, T_corrected);
    else
        fprintf('✓ 速度误差在可接受范围内\n');
    end
end

fprintf('\n===== 结论 =====\n');
fprintf('5次多项式轨迹的最大速度公式 v_max ≈ 15/8 * dist / T 是近似估算。\n');
fprintf('实际最大速度可能略小于期望值，特别是当加速度限制主导时。\n');
fprintf('建议：在 generate_moveJ_trajectory 中添加速度验证和迭代调整。\n');
