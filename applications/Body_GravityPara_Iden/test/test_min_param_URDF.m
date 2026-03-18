%% test_min_param_URDF  最小参数集正确性验收
%
% 验收逻辑（两步）：
%
%  Step 1 — 辨识自洽性（纯数值）
%    用随机 (q,qd,qdd) 生成 τ_sim = inverseDynamics(robot, q, qd, qdd)，
%    再把这批数据包装成 avg_data，调 identify_min 辨识出 X_hat，
%    检查 Y_min * X_hat ≈ τ_sim（残差 RMSE 应在数值误差量级 < 1e-6 N·m）。
%    若通过，说明 rref 列选取、Y_min 组装、最小二乘求解均正确。
%
%  Step 2 — 参数一致性（θ 投影）
%    从 URDF 读取全参向量 θ（get_limb_theta_from_URDF），
%    用相同数据得 Y_full，令 X_theta = Y_min \ (Y_full * θ) 得理论最小参数，
%    检查 X_hat ≈ X_theta（误差 < 1e-6 N·m 量级 in L∞）。
%    若通过，说明辨识出的最小参数与 URDF 真值在同一线性组合下一致。
%
% 运行：在 MATLAB 中 cd 到 Body_GravityPara_Iden 目录，或把该目录及 robot_model 加入路径后运行。

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

limb       = 'left_leg';
para_order = 1;
tol_step1  = 1e-4;   % N·m，Step 1 RMSE 上限（辨识自洽性）
tol_step2  = 1e-4;   % Step 2 X_hat 与理论值 L∞ 误差上限

[robot_limb, n] = get_e1_limb_robot(limb);
theta_urdf = get_limb_theta_from_URDF(robot_limb, para_order);

%% 生成仿真数据
rng(42);
M = 60;                   % 样本数（>=2×p_min，保证 A 行数足够）
q_rad   = 0.6;
qd_rad  = 0.5;
qdd_rad = 0.8;

q_all   = (rand(M, n) - 0.5) * 2 * q_rad;
qd_all  = (rand(M, n) - 0.5) * 2 * qd_rad;
qdd_all = (rand(M, n) - 0.5) * 2 * qdd_rad;

% 第 1 行固定零（静力学基准）
q_all(1,:)   = zeros(1, n);
qd_all(1,:)  = zeros(1, n);
qdd_all(1,:) = zeros(1, n);

% 用 URDF inverseDynamics 生成仿真力矩（作为"测量"τ）
tau_all = zeros(M, n);
for k = 1:M
    tau_all(k,:) = inverseDynamics(robot_limb, q_all(k,:), qd_all(k,:), qdd_all(k,:))';
end

%% 包装成 avg_data 结构（identify_min 所需格式）
avg_data.q_bar   = q_all;
avg_data.qd_bar  = qd_all;
avg_data.qdd_bar = qdd_all;
avg_data.tau_bar = tau_all;
% tau_std 置 0 → identify_min 会退回等权 OLS（正好不依赖 tau_std 来源）
avg_data.tau_std = zeros(M, n);

%% ===== Step 1：辨识自洽性 =====
fprintf('===== test_min_param_URDF =====\n');
fprintf('肢体: %s, 样本数: %d, para_order: %d\n', limb, M, para_order);

[X_hat, index_base, metrics] = identify_min(avg_data, limb, 'para_order', para_order, 'use_wls', false);

p_min = numel(X_hat);
rmse_max_step1 = max(metrics.rmse_per_joint);

fprintf('\n--- Step 1: 辨识自洽性（Y_min * X_hat ≈ τ_sim）---\n');
fprintf('最小参数维度 p_min = %d\n', p_min);
fprintf('各关节 RMSE (N·m): '); fprintf(' %.4e', metrics.rmse_per_joint); fprintf('\n');
fprintf('max RMSE = %.4e N·m\n', rmse_max_step1);
if rmse_max_step1 <= tol_step1
    fprintf('Step 1 通过 (tol = %.1e N·m)\n', tol_step1);
else
    fprintf('Step 1 未通过 (tol = %.1e N·m) — 检查 rref / identify_min 实现\n', tol_step1);
end

%% ===== Step 2：参数一致性（X_hat vs θ 理论投影）=====
fprintf('\n--- Step 2: 参数一致性（X_hat vs θ 投影）---\n');

Y_full = ReMatrix_E1_limb_URDF(limb, q_all, qd_all, qdd_all, 1, para_order);
Y_min  = Y_full(:, index_base);

% 理论最小参数 = Y_min \ (Y_full * θ)
tau_theta = Y_full * theta_urdf;           % 应与 tau_all(:) 吻合（Step 1 前置条件）
X_theta   = Y_min \ tau_theta;             % 理论最小参数

err_param = abs(X_hat - X_theta);
max_err_param = max(err_param);
fprintf('X_hat 与 θ_URDF 投影值 L∞ 误差 = %.4e\n', max_err_param);
if max_err_param <= tol_step2
    fprintf('Step 2 通过 (tol = %.1e)\n', tol_step2);
else
    fprintf('Step 2 未通过 (tol = %.1e) — 检查 theta_urdf / Y_full 列顺序是否与 identify_min 一致\n', tol_step2);
    [worst, idx] = max(err_param);
    fprintf('最差参数: index_base(%d)=%d, X_hat=%.6f, X_theta=%.6f, diff=%.4e\n', ...
        idx, index_base(idx), X_hat(idx), X_theta(idx), worst);
end

fprintf('===============================\n');

%% ===== 图：τ_sim vs τ_pred（Step 1 可视化）=====
t_equiv = (1:M)';

figure('Name', 'tau_sim vs tau_pred (Y_min * X_hat)');
plot_compare_6dof(t_equiv, [metrics.tau_meas, metrics.tau_pred], 'torque', ...
    {'\tau_{sim} (inverseDynamics)', '\tau_{pred} (Y_{min}X̂)'});
sgtitle('最小参数验收 Step 1：\tau_{sim} vs Y_{min}\hat{X}');

% 图2：参数误差
figure('Name', 'X_hat vs X_theta');
stem(err_param, 'filled', 'MarkerSize', 4);
xlabel('参数索引 i (in index\_base)');
ylabel('|X\_hat - X\_theta|');
title(sprintf('最小参数验收 Step 2：X\_hat 与 URDF θ 投影偏差（max=%.2e）', max_err_param));
grid on;
