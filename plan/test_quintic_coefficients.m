% test_quintic_coefficients  测试5次多项式系数推导
% 验证加速段和减速段的系数是否正确

clear; clc;

%% 测试加速段系数
fprintf('===== 测试加速段系数 =====\n');
q_start = 0;
q_end = 1.0;  % 距离1 rad
delta_q = q_end - q_start;
v_target = 2.0;  % 目标速度 2 rad/s
T = 1.0;  % 时间1秒
v_target_normalized = v_target * T;  % 归一化速度 = 2.0

% 计算系数
a0 = q_start;
a1 = 0;
a2 = 0;
a3 = 10 * delta_q - 4 * v_target_normalized;
a4 = -15 * delta_q + 7 * v_target_normalized;
a5 = 6 * delta_q - 3 * v_target_normalized;

fprintf('系数: a0=%.3f, a1=%.3f, a2=%.3f, a3=%.3f, a4=%.3f, a5=%.3f\n', ...
    a0, a1, a2, a3, a4, a5);

% 验证边界条件
s = 0;
q_0 = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5;
qd_0 = a1 + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4;
qdd_0 = 2*a2 + 6*a3*s + 12*a4*s^2 + 20*a5*s^3;

s = 1;
q_1 = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5;
qd_1 = a1 + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4;
qdd_1 = 2*a2 + 6*a3*s + 12*a4*s^2 + 20*a5*s^3;

fprintf('\n边界条件验证（s=0）:\n');
fprintf('  q(0) = %.6f (期望 %.6f)\n', q_0, q_start);
fprintf('  qd(0) = %.6f (期望 0.0)\n', qd_0);
fprintf('  qdd(0) = %.6f (期望 0.0)\n', qdd_0);

fprintf('\n边界条件验证（s=1）:\n');
fprintf('  q(1) = %.6f (期望 %.6f)\n', q_1, q_end);
fprintf('  qd(1) = %.6f (期望 %.6f，实际速度=qd/T=%.6f rad/s)\n', ...
    qd_1, v_target_normalized, qd_1/T);
fprintf('  qdd(1) = %.6f (期望 0.0)\n', qdd_1);

fprintf('\n验证方程组:\n');
eq1 = a3 + a4 + a5 - delta_q;
eq2 = 3*a3 + 4*a4 + 5*a5 - v_target_normalized;
eq3 = 6*a3 + 12*a4 + 20*a5;
fprintf('  a3 + a4 + a5 - delta_q = %.6e (应为0)\n', eq1);
fprintf('  3*a3 + 4*a4 + 5*a5 - v_target_normalized = %.6e (应为0)\n', eq2);
fprintf('  6*a3 + 12*a4 + 20*a5 = %.6e (应为0)\n', eq3);

if abs(eq1) < 1e-10 && abs(eq2) < 1e-10 && abs(eq3) < 1e-10 && ...
   abs(qd_1/T - v_target) < 1e-6
    fprintf('\n✓ 加速段系数正确！\n');
else
    fprintf('\n✗ 加速段系数有误！\n');
end

%% 测试减速段系数
fprintf('\n===== 测试减速段系数 =====\n');
q_start_decel = 1.0;
q_end_decel = 0.0;
delta_q_decel = q_end_decel - q_start_decel;
v_start = 2.0;  % 起始速度 2 rad/s
T_decel = 1.0;
v_start_normalized = v_start * T_decel;  % 归一化速度 = 2.0

% 计算系数
a0_dec = q_start_decel;
a1_dec = v_start_normalized;
a2_dec = 0;
a3_dec = 10 * delta_q_decel - 6 * v_start_normalized;
a4_dec = -15 * delta_q_decel + 8 * v_start_normalized;
a5_dec = 6 * delta_q_decel - 3 * v_start_normalized;

fprintf('系数: a0=%.3f, a1=%.3f, a2=%.3f, a3=%.3f, a4=%.3f, a5=%.3f\n', ...
    a0_dec, a1_dec, a2_dec, a3_dec, a4_dec, a5_dec);

% 验证边界条件
s = 0;
q_0_dec = a0_dec + a1_dec*s + a2_dec*s^2 + a3_dec*s^3 + a4_dec*s^4 + a5_dec*s^5;
qd_0_dec = a1_dec + 2*a2_dec*s + 3*a3_dec*s^2 + 4*a4_dec*s^3 + 5*a5_dec*s^4;
qdd_0_dec = 2*a2_dec + 6*a3_dec*s + 12*a4_dec*s^2 + 20*a5_dec*s^3;

s = 1;
q_1_dec = a0_dec + a1_dec*s + a2_dec*s^2 + a3_dec*s^3 + a4_dec*s^4 + a5_dec*s^5;
qd_1_dec = a1_dec + 2*a2_dec*s + 3*a3_dec*s^2 + 4*a4_dec*s^3 + 5*a5_dec*s^4;
qdd_1_dec = 2*a2_dec + 6*a3_dec*s + 12*a4_dec*s^2 + 20*a5_dec*s^3;

fprintf('\n边界条件验证（s=0）:\n');
fprintf('  q(0) = %.6f (期望 %.6f)\n', q_0_dec, q_start_decel);
fprintf('  qd(0) = %.6f (期望 %.6f，实际速度=qd/T=%.6f rad/s)\n', ...
    qd_0_dec, v_start_normalized, qd_0_dec/T_decel);
fprintf('  qdd(0) = %.6f (期望 0.0)\n', qdd_0_dec);

fprintf('\n边界条件验证（s=1）:\n');
fprintf('  q(1) = %.6f (期望 %.6f)\n', q_1_dec, q_end_decel);
fprintf('  qd(1) = %.6f (期望 0.0)\n', qd_1_dec);
fprintf('  qdd(1) = %.6f (期望 0.0)\n', qdd_1_dec);

fprintf('\n验证方程组:\n');
eq1_dec = a3_dec + a4_dec + a5_dec - (delta_q_decel - v_start_normalized);
eq2_dec = a1_dec + 3*a3_dec + 4*a4_dec + 5*a5_dec;
eq3_dec = 6*a3_dec + 12*a4_dec + 20*a5_dec;
fprintf('  a3 + a4 + a5 - (delta_q - v_start) = %.6e (应为0)\n', eq1_dec);
fprintf('  a1 + 3*a3 + 4*a4 + 5*a5 = %.6e (应为0)\n', eq2_dec);
fprintf('  6*a3 + 12*a4 + 20*a5 = %.6e (应为0)\n', eq3_dec);

if abs(eq1_dec) < 1e-10 && abs(eq2_dec) < 1e-10 && abs(eq3_dec) < 1e-10 && ...
   abs(qd_0_dec/T_decel - v_start) < 1e-6 && abs(qd_1_dec) < 1e-6
    fprintf('\n✓ 减速段系数正确！\n');
else
    fprintf('\n✗ 减速段系数有误！\n');
end
