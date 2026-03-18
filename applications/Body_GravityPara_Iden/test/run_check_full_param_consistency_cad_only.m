%% run_check_full_param_consistency_cad_only  CAD 全参链路一致性自检（不依赖辨识结果）
%
% 仅验证 URDF ↔ ReMatrix ↔ π_cad 定义链路，与 min_param_id_result.mat 无关。
% 自检 1：Y_full*π_cad vs τ_urdf（逆动力学）
% 自检 2：forward_dynamics_full(π_cad) vs FD_urdf（正动力学）
% 自检 3：单参数扰动后 τ_full 是否变化（自动选一列可观参数，不绑死 link1 mass）
%
% 轨迹来源：trajectory_source = 'random'（默认，无需文件）或 'csv'（需 data/excitation 下 CSV）。
% 阶段 A 通过本脚本即可判断全参链路是否通，再决定是否跑辨识与 β→π。
%
% 用法：cd 到 Body_GravityPara_Iden/test，运行 run_check_full_param_consistency_cad_only

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

limb = 'left_leg';
para_order = 1;

% 轨迹来源：'random' = 随机点，无需文件；'csv' = 从 CSV 窗内取
trajectory_source = 'random';
n_random = 80;   % random 时点数
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
csv_opts = struct('t_start_s', 2.1, 't_end_s', 4.1, 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);

%% 生成或加载轨迹（不依赖 mat）
if strcmpi(trajectory_source, 'random')
    rng(42);
    % 关节限位（左腿 6 自由度，避免随机跑到不合理姿态）
    q_min = [-0.5, -1.0, -1.2, -1.5, -1.0, -0.5];
    q_max = [ 0.5,  1.0,  1.2,  1.5,  1.0,  0.5];
    if numel(q_min) < n, q_min = [q_min, -0.5*ones(1,n-numel(q_min))]; end
    if numel(q_max) < n, q_max = [q_max,  0.5*ones(1,n-numel(q_max))]; end
    q_bar   = q_min + rand(n_random, n) .* (q_max(1:n) - q_min(1:n));
    qd_bar  = (rand(n_random, n) - 0.5) * 1.0;
    qdd_bar = (rand(n_random, n) - 0.5) * 1.0;
    tau_bar = zeros(n_random, n);
    for k = 1:n_random
        tau_bar(k,:) = inverseDynamics(robot_limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:))';
    end
    M = n_random;
    t_plot = (1:M)';
    fprintf('轨迹: 随机 %d 点（无文件）\n', M);
else
    if ~isfile(csv_file)
        error('CSV 不存在: %s', csv_file);
    end
    data = read_leg_joint_csv(csv_file);
    t = data.time(:);
    q = data.pos_leg_l; qd = data.vel_leg_l; tau = data.torque_leg_l;
    avg_data = continuous_window_id_data(t, q, qd, tau, csv_opts);
    q_bar   = avg_data.q_bar;
    qd_bar  = avg_data.qd_bar;
    qdd_bar = avg_data.qdd_bar;
    tau_bar = avg_data.tau_bar;
    M = size(q_bar, 1);
    t_plot = avg_data.t_equiv(:);
    fprintf('轨迹: CSV 窗内 %d 点\n', M);
end

%% 构造 Y_full（不依赖 index_base / mat；重力与 inverseDynamics 一致，均来自 get_e1_limb_robot → get_e1_gravity）
p = numel(pi_cad);
Y_full = zeros(M * n, p);
for k = 1:M
    rows = (k-1) * n + (1:n);
    Y_full(rows, :) = ReMatrix_E1_limb_URDF(limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, para_order);
end

fprintf('===== CAD 全参链路一致性自检（不依赖辨识结果） =====\n');
fprintf('轨迹点数 M = %d，自由度 n = %d，全参 p = %d\n', M, n, numel(pi_cad));

%% 自检 1：τ_cad_full vs τ_urdf
tau_urdf = zeros(M, n);
for k = 1:M
    tau_urdf(k,:) = inverseDynamics(robot_limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:))';
end
tau_cad_full = reshape(Y_full * pi_cad, n, M)';

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
    fprintf('  → 建议立刻运行 run_diagnose_pi_Y_columns 或 run_check_pi_order_vs_urdf / run_check_regressor_column_norms。\n');
end

fig1 = figure('Name', 'CAD自检1_逆动力学_τ对比', 'Position', [50 50 1200 750]);
for j = 1:n
    subplot(2, 3, j);
    plot(t_plot, tau_cad_full(:,j), 'b-', 'DisplayName', 'Y_{full}\pi_{cad}'); hold on;
    plot(t_plot, tau_urdf(:,j), 'r--', 'DisplayName', '\tau_{urdf}');
    plot(t_plot, err_id(:,j), 'k:', 'DisplayName', '误差');
    xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('关节 %d', j)); legend('Location', 'best'); grid on; hold off;
end
sgtitle(fig1, 'CAD 自检 1：\tau_{cad,full} vs \tau_{urdf}');

%% 自检 2：FD(π_cad) vs FD_urdf
n_fd_check = min(M, 50);
idx_check  = round(linspace(1, M, n_fd_check));
qdd_fd_cad = zeros(n_fd_check, n);
qdd_urdf   = zeros(n_fd_check, n);
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
rmse_per_joint = sqrt(mean(err_fd.^2, 1));
fprintf('  各关节 qdd 误差 RMSE: '); fprintf(' %.4e', rmse_per_joint); fprintf(' rad/s^2\n');
if rel_err_2 < 1e-5
    fprintf('  通过：forward_dynamics_full(π_cad) 与 URDF 正动力学一致。\n');
else
    fprintf('  未通过：请检查 ReMatrix 构造 M/h 与 URDF 正动力学是否同源。\n');
    fprintf('  → 建议立刻运行 run_diagnose_pi_Y_columns。\n');
end

t_fd = t_plot(idx_check);
fig2 = figure('Name', 'CAD自检2_正动力学_qdd对比', 'Position', [80 80 1200 750]);
for j = 1:n
    subplot(2, 3, j);
    plot(t_fd, qdd_urdf(:,j), 'b-', 'DisplayName', 'qdd_{urdf}'); hold on;
    plot(t_fd, qdd_fd_cad(:,j), 'r--', 'DisplayName', 'qdd_{fd,cad}');
    plot(t_fd, err_fd(:,j), 'k:', 'DisplayName', '误差');
    xlabel('t (s)'); ylabel('rad/s^2'); title(sprintf('关节 %d', j)); legend('Location', 'best'); grid on; hold off;
end
sgtitle(fig2, 'CAD 自检 2：FD(\pi_{cad}) vs FD_{urdf}');

%% 自检 3：单参数扰动（自动选可观列，避免 link1 m 结构不可观时误判）
col_norms = sqrt(sum(Y_full.^2, 1));
cand = 1:10;
nz = cand(col_norms(cand) > 1e-10);
if ~isempty(nz)
    [~, ii] = max(col_norms(nz));
    j_test = nz(ii);
else
    [~, j_test] = max(col_norms);
end
pi_perturb = pi_cad(:);
delta = 0.1 * pi_cad(j_test);
if abs(delta) < 1e-8
    delta = 1e-6;
end
pi_perturb(j_test) = pi_perturb(j_test) + delta;
tau_perturb = reshape(Y_full * pi_perturb, n, M)';
diff_tau = tau_perturb - tau_cad_full;
max_diff_3 = max(abs(diff_tau(:)));
mean_diff_3 = mean(abs(diff_tau(:)));
std_per_joint = std(diff_tau, 0, 1);

param_names_10 = {'m','mx','my','mz','Ixx','Ixy','Ixz','Iyy','Iyz','Izz'};
link_j = floor((j_test-1)/10) + 1;
param_j = mod(j_test-1, 10) + 1;

fprintf('\n--- 自检 3：单参数扰动（第 %d 列 = link%d %s）后 τ_full 变化 ---\n', j_test, link_j, param_names_10{param_j});
fprintf('  ||Y(:,%d)|| = %.4e\n', j_test, col_norms(j_test));
fprintf('  delta = %.4e\n', delta);
fprintf('  max|Δτ| = %.4e N·m\n', max_diff_3);
fprintf('  mean|Δτ| = %.4e N·m\n', mean_diff_3);
fprintf('  各关节 Δτ 标准差: '); fprintf(' %.4e', std_per_joint); fprintf(' N·m\n');
if col_norms(j_test) <= 1e-10
    fprintf('  列可见性：该列在当前轨迹下不可观（||Y(:,j)||≈0），属结构/轨迹导致，非必然错误。\n');
elseif max_diff_3 <= 1e-12
    fprintf('  列可见性：该列可观但扰动后 Δτ≈0，请排查 Y 组装或 π 顺序。\n');
else
    fprintf('  列可见性：该列可观且扰动后 τ 变化正常。\n');
end

fig3 = figure('Name', 'CAD自检3_单参扰动_Δτ', 'Position', [110 110 700 320]);
subplot(1, 2, 1);
bar(1:n, max(abs(diff_tau), [], 1), 'FaceColor', [0.4 0.6 0.9]); xlabel('关节'); ylabel('max|\Delta\tau|'); title('各关节 max|Δτ|'); grid on;
subplot(1, 2, 2);
bar(1:n, mean(abs(diff_tau), 1), 'FaceColor', [0.6 0.8 0.6]); xlabel('关节'); ylabel('mean|\Delta\tau|'); title('各关节 mean|Δτ|'); grid on;
sgtitle(fig3, sprintf('CAD 自检 3：第%d列(link%d %s)扰动后 \\Delta\\tau', j_test, link_j, param_names_10{param_j}), 'Interpreter', 'none');

%% 汇总与门槛（关键停止仅看自检 1、2；自检 3 为列可见性报告）
pass1 = rel_err_1 < 1e-6;
pass2 = rel_err_2 < 1e-5;
pass3 = col_norms(j_test) > 1e-10 && max_diff_3 > 1e-12;
cad_ok = pass1 && pass2;

fprintf('\n========== CAD 全参链路 汇总 ==========\n');
fprintf('  自检 1 (ID 一致):     rel_err = %.4e  %s\n', rel_err_1, iif(pass1, '通过', '未通过'));
fprintf('  自检 2 (FD 一致):     rel_err = %.4e  %s\n', rel_err_2, iif(pass2, '通过', '未通过'));
fprintf('  自检 3 (列可见性):    列 j=%d, max|Δτ| = %.4e  %s\n', j_test, max_diff_3, iif(pass3, '可观', '不可观/未激励'));
if cad_ok
    fprintf('\n→ CAD 全参链路通过。可进入阶段 B（真实辨识）与阶段 C（FD 验证）。\n');
else
    fprintf('\n→ 【关键停止】CAD 全参链路未通过（自检 1 或 2 未过）。禁止跑 β→π、FD_full、三模型对比；先跑 run_diagnose_pi_Y_columns 修链路。\n');
end

function s = iif(cond, a, b)
if cond, s = a; else, s = b; end
end
