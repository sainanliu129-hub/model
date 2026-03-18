%% run_check_full_param_consistency  全参数定义链路一致性自检（三步，用 mat 轨迹）
%
% 与 run_check_full_param_consistency_cad_only 自检内容相同，但轨迹来自 min_param_id_result.mat，
% 便于与辨识结果同轨迹对比。阶段 A 请先跑 cad_only（不依赖 mat）；本脚本用于阶段 D 完整校核。
%
% 自检 1：τ_cad_full = Y_full*π_cad 与 τ_urdf 是否一致
% 自检 2：forward_dynamics_full(π_cad) 与 forwardDynamics(URDF) 是否一致
% 自检 3：单参数扰动（link1 质量 +10%）后 τ_full 是否变化
%
% 前置：min_param_id_result.mat 已存在。
% 用法：cd 到 Body_GravityPara_Iden/test，运行 run_check_full_param_consistency

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

limb = 'left_leg';
para_order = 1;

%% 加载轨迹与构造 Y_full、π_cad
result_mat = fullfile(app_root, 'min_param_id_result.mat');
if ~isfile(result_mat)
    error('请先运行 run_min_param_id_from_csv 生成 %s', result_mat);
end
ld = load(result_mat);
avg_data = ld.avg_data;
q_bar   = avg_data.q_bar;
qd_bar  = avg_data.qd_bar;
qdd_bar = avg_data.qdd_bar;
tau_bar = avg_data.tau_bar;
[M, n] = size(q_bar);

[~, Y_full, ~, ~] = build_K_from_regressor(limb, q_bar, qd_bar, qdd_bar, ld.index_base, para_order);
[robot_limb, ~] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);

fprintf('===== 全参数定义链路一致性自检 =====\n');
fprintf('轨迹点数 M = %d，自由度 n = %d，全参 p = %d\n', M, n, numel(pi_cad));

%% 自检 1：τ_cad_full = Y_full*π_cad 与 τ_urdf 是否一致
tau_urdf = zeros(M, n);
for k = 1:M
    tau_urdf(k,:) = inverseDynamics(robot_limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:))';
end
tau_cad_full = reshape(Y_full * pi_cad, n, M)';   % (M*n)×1 -> M×n

err_id = tau_cad_full - tau_urdf;
rel_err_1 = norm(err_id, 'fro') / (norm(tau_urdf, 'fro') + 1e-20);
max_abs_1 = max(abs(err_id(:)));

fprintf('\n--- 自检 1：τ_full(π_cad) vs τ_urdf（逆动力学） ---\n');
fprintf('  ||τ_cad_full - τ_urdf||_F / ||τ_urdf||_F = %.4e\n', rel_err_1);
fprintf('  max|τ_cad_full - τ_urdf| = %.4e N·m\n', max_abs_1);
if rel_err_1 < 1e-6
    fprintf('  通过：Y_full*π_cad 与 URDF 逆动力学一致。\n');
else
    fprintf('  未通过：请检查 get_limb_theta_from_URDF / ReMatrix 与 URDF 参数顺序与惯量展开。\n');
end

% 图1：自检 1 — τ_cad_full vs τ_urdf 及误差
t_plot = (1:M)';
if isfield(avg_data, 't_equiv') && numel(avg_data.t_equiv) == M
    t_plot = avg_data.t_equiv(:);
end
fig1 = figure('Name', '自检1_逆动力学_τ对比', 'Position', [50 50 1200 750]);
for j = 1:n
    subplot(2, 3, j);
    plot(t_plot, tau_cad_full(:,j), 'b-', 'DisplayName', 'Y_{full}\pi_{cad}'); hold on;
    plot(t_plot, tau_urdf(:,j), 'r--', 'DisplayName', '\tau_{urdf}');
    plot(t_plot, err_id(:,j), 'k:', 'DisplayName', '误差');
    xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('关节 %d', j)); legend('Location', 'best'); grid on; hold off;
end
sgtitle(fig1, '自检 1：\tau_{cad,full} = Y_{full}\pi_{cad} vs \tau_{urdf}（逆动力学）');

%% 自检 2：forward_dynamics_full(π_cad) vs forwardDynamics(URDF)
n_fd_check = min(M, 50);   % 抽样点数，避免过慢
qdd_fd_cad = zeros(n_fd_check, n);
qdd_urdf   = zeros(n_fd_check, n);
idx_check  = round(linspace(1, M, n_fd_check));

for ii = 1:n_fd_check
    k = idx_check(ii);
    qdd_fd_cad(ii,:) = forward_dynamics_full(q_bar(k,:), qd_bar(k,:), tau_bar(k,:), pi_cad, limb, para_order)';
    qdd_urdf(ii,:)   = forwardDynamics(robot_limb, q_bar(k,:), qd_bar(k,:), tau_bar(k,:));
end

err_fd = qdd_fd_cad - qdd_urdf;
rel_err_2 = norm(err_fd, 'fro') / (norm(qdd_urdf, 'fro') + 1e-20);
max_abs_2 = max(abs(err_fd(:)));

fprintf('\n--- 自检 2：qdd = FD(π_cad) vs FD_urdf（正动力学，抽样 %d 点） ---\n', n_fd_check);
fprintf('  ||qdd_fd_cad - qdd_urdf||_F / ||qdd_urdf||_F = %.4e\n', rel_err_2);
fprintf('  max|qdd_fd_cad - qdd_urdf| = %.4e rad/s^2\n', max_abs_2);
if rel_err_2 < 1e-5
    fprintf('  通过：forward_dynamics_full(π_cad) 与 URDF 正动力学一致。\n');
else
    fprintf('  未通过：请检查 ReMatrix 构造 M/h 与 URDF 正动力学是否同源。\n');
end

% 图2：自检 2 — qdd_fd_cad vs qdd_urdf（抽样点）
t_fd = (1:n_fd_check)';
if isfield(avg_data, 't_equiv') && numel(avg_data.t_equiv) == M
    t_fd = avg_data.t_equiv(idx_check);
end
fig2 = figure('Name', '自检2_正动力学_qdd对比', 'Position', [80 80 1200 750]);
for j = 1:n
    subplot(2, 3, j);
    plot(t_fd, qdd_urdf(:,j), 'b-', 'DisplayName', 'qdd_{urdf}'); hold on;
    plot(t_fd, qdd_fd_cad(:,j), 'r--', 'DisplayName', 'qdd_{fd,cad}');
    plot(t_fd, err_fd(:,j), 'k:', 'DisplayName', '误差');
    xlabel('t (s)'); ylabel('rad/s^2'); title(sprintf('关节 %d', j)); legend('Location', 'best'); grid on; hold off;
end
sgtitle(fig2, '自检 2：FD(\pi_{cad}) vs FD_{urdf}（正动力学，抽样点）');

%% 自检 3：单参数扰动（第 1 个 link 质量 +10%）后 τ 变化是否合理
pi_perturb = pi_cad(:);
idx_mass1 = 1;   % 第 1 个 link 的 m
pi_perturb(idx_mass1) = pi_cad(idx_mass1) * 1.1;

tau_perturb = reshape(Y_full * pi_perturb, n, M)';
diff_tau = tau_perturb - tau_cad_full;
max_diff_3 = max(abs(diff_tau(:)));
mean_diff_3 = mean(abs(diff_tau(:)));
std_per_joint = std(diff_tau, 0, 1);

fprintf('\n--- 自检 3：单参数扰动（link1 质量 +10%%）后 τ_full 变化 ---\n');
fprintf('  π_cad(1) (link1 mass) = %.6e kg\n', pi_cad(1));
fprintf('  max|τ_perturb - τ_cad_full| = %.4e N·m\n', max_diff_3);
fprintf('  mean|Δτ| = %.4e N·m\n', mean_diff_3);
fprintf('  各关节 Δτ 标准差: '); fprintf(' %.4e', std_per_joint); fprintf(' N·m\n');
if max_diff_3 <= 1e-12
    fprintf('  未通过：单参数扰动后逆动力学输出不变，说明全参数向量与回归矩阵 Y 的参数对应关系存在错误（或 Y 第 1 列为零）。请运行 run_diagnose_pi_Y_columns 排查。\n');
elseif max_diff_3 < 1e3 && mean_diff_3 < 1e2
    fprintf('  合理：扰动后力矩变化量级正常，无异常尖峰。\n');
else
    fprintf('  注意：变化较大或存在尖峰时，需排查该参数在 Y 中对应列的解释是否一致。\n');
end

% 图3：自检 3 — 各关节 Δτ 的 max/mean（扰动后应有变化）
fig3 = figure('Name', '自检3_单参扰动_Δτ', 'Position', [110 110 700 320]);
subplot(1, 2, 1);
bar(1:n, max(abs(diff_tau), [], 1), 'FaceColor', [0.4 0.6 0.9]); xlabel('关节'); ylabel('max|\Delta\tau| (N\cdotm)'); title('各关节 max|Δτ|'); grid on;
subplot(1, 2, 2);
bar(1:n, mean(abs(diff_tau), 1), 'FaceColor', [0.6 0.8 0.6]); xlabel('关节'); ylabel('mean|\Delta\tau| (N\cdotm)'); title('各关节 mean|Δτ|'); grid on;
sgtitle(fig3, '自检 3：link1 质量 +10\% 后 \Delta\tau = \tau_{perturb} - \tau_{cad,full}');

%% 汇总
fprintf('\n========== 汇总 ==========\n');
fprintf('  自检 1 (ID 一致):     rel_err = %.4e  %s\n', rel_err_1, iif(rel_err_1 < 1e-6, '通过', '未通过'));
fprintf('  自检 2 (FD 一致):     rel_err = %.4e  %s\n', rel_err_2, iif(rel_err_2 < 1e-5, '通过', '未通过'));
pass3 = max_diff_3 > 1e-12 && (max_diff_3 < 1e3 && mean_diff_3 < 1e2);
fprintf('  自检 3 (扰动有效且合理): max|Δτ| = %.4e  %s\n', max_diff_3, iif(pass3, '通过', '未通过'));
fprintf('若 1、2 均通过，可进行 Wensing LMI/SDP 恢复；否则先修参数定义链路。\n');

function s = iif(cond, a, b)
if cond, s = a; else, s = b; end
end
