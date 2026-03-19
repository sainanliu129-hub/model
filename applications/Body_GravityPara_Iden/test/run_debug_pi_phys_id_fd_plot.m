%% run_debug_pi_phys_id_fd_plot
% 单独调试：FD 导向 full 参数辨识（pi_fd）并做 ID/FD 对比图。
% 说明：脚本重构为“薄流程”，核心计算复用 dispatch 与 principle 函数，减少重复代码。

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 0) 开关与参数
save_result = true;
out_dir = fullfile(app_root, 'build');
out_mat = fullfile(out_dir, 'debug_pi_fd_latest.mat');

run_pi_fd = true;
M_fd_max = 500;

opts_fd = struct();
opts_fd.w_tau = 3;
opts_fd.w_qdd = 5;
opts_fd.w_cad = 1;
opts_fd.w_M = 1;
opts_fd.m_min_frac = 0.5;
opts_fd.m_max_frac = 1.5;
opts_fd.delta_c = 0.15;
opts_fd.eps_I = 0;
opts_fd.eps_M = 1e-6;
opts_fd.algorithm = 'sqp';
opts_fd.display = 'iter';
opts_fd.max_iter = 50;
opts_fd.MaxFunctionEvaluations = 3000;
opts_fd.n_qdd = 20;
opts_fd.n_reg = 8;
opts_fd.debug_constraints = true;
opts_fd.debug_constraints_every = 50;

%% 1) 载入最小参数辨识结果
result_mat = fullfile(app_root, 'min_param_id_result.mat');
if ~isfile(result_mat)
    error('未找到 %s，请先运行 run_min_param_id_from_csv。', result_mat);
end
ld = load(result_mat);
X_hat = ld.X_hat(:);
index_base = ld.index_base(:);
avg_data = ld.avg_data;

limb = 'left_leg';
para_order = 1;
[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);

q = avg_data.q_bar;
qd = avg_data.qd_bar;
qdd = avg_data.qdd_bar;
tau_meas = avg_data.tau_bar;
t = (1:size(q, 1)).';
if isfield(avg_data, 't_equiv') && ~isempty(avg_data.t_equiv)
    t = avg_data.t_equiv(:);
end
M = size(q, 1);

Y_full = ReMatrix_E1_limb_URDF(limb, q, qd, qdd, 1, para_order);

%% 2) FD 导向 full 参数辨识（可选）
pi_fd = [];
info_fd = struct();
if run_pi_fd
    fprintf('\n===== FD 导向 full 参数辨识（pi_fd） =====\n');
    n_qdd = min(opts_fd.n_qdd, M);
    n_reg = min(opts_fd.n_reg, M);
    opts_fd.idx_qdd = round(linspace(1, M, n_qdd)).';
    opts_fd.idx_reg = round(linspace(1, M, n_reg)).';
    opts_fd = rmfield(opts_fd, {'n_qdd', 'n_reg'});

    [pi_fd, info_fd] = identify_full_params_for_fd(q, qd, qdd, tau_meas, pi_cad, limb, para_order, opts_fd);
    fprintf('  exitflag=%d, fval=%.3e, rmse_tau=%.4e, rmse_qdd=%.4e\n', ...
        info_fd.exitflag, info_fd.fval, info_fd.rmse_tau, info_fd.rmse_qdd);
end

if save_result
    if ~isfolder(out_dir), mkdir(out_dir); end
    S = struct('pi_fd', pi_fd, 'info_fd', info_fd, 'pi_cad', pi_cad, ...
        'X_hat', X_hat, 'index_base', index_base, 'limb', limb, 'para_order', para_order);
    save(out_mat, '-struct', 'S');
end

%% 3) 逆动力学对比：tau_meas vs Y*pi
tau_id_cad = (Y_full * pi_cad).';
tau_id_min = zeros(M, n);
for k = 1:M
    model_min = struct('type', 'min', 'limb', limb, 'para_order', para_order, ...
        'X_hat', X_hat, 'index_base', index_base);
    tau_id_min(k, :) = inverse_dynamics_dispatch(q(k, :), qd(k, :), qdd(k, :), model_min).';
end
if ~isempty(pi_fd)
    tau_id_fd = (Y_full * pi_fd(:)).';
end

fprintf('\n===== 逆动力学 RMSE（vs tau_meas） =====\n');
fprintf('  CAD: '); fprintf(' %.3f', sqrt(mean((tau_id_cad - tau_meas).^2, 1))); fprintf('\n');
fprintf('  MIN: '); fprintf(' %.3f', sqrt(mean((tau_id_min - tau_meas).^2, 1))); fprintf('\n');
if ~isempty(pi_fd)
    fprintf('  FD : '); fprintf(' %.3f', sqrt(mean((tau_id_fd - tau_meas).^2, 1))); fprintf('\n');
end

if ~isempty(pi_fd)
    plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
        [tau_id_fd, tau_id_cad, tau_id_min], {'Y\pi_{fd}', 'Y\pi_{cad}', 'Y_{min}X_{hat}'}, ...
        'torque', '逆动力学调试');
else
    plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
        [tau_id_cad, tau_id_min], {'Y\pi_{cad}', 'Y_{min}X_{hat}'}, ...
        'torque', '逆动力学调试');
end

%% 4) 正动力学对比：qdd_ref vs FD(...)
M_fd = min(M, M_fd_max);
t_fd = t(1:M_fd);
qd_ref = qd(1:M_fd, :);
qdd_ref = qdd(1:M_fd, :);
qdd_cad = zeros(M_fd, n);
qdd_min = zeros(M_fd, n);
qdd_fd = zeros(M_fd, n);

model_cad = struct('type', 'urdf', 'limb', limb, 'para_order', para_order);
model_min = struct('type', 'min', 'limb', limb, 'para_order', para_order, ...
    'X_hat', X_hat, 'index_base', index_base);
if ~isempty(pi_fd)
    model_fd = struct('type', 'full', 'limb', limb, 'para_order', para_order, 'pi_vec', pi_fd(:));
end

for k = 1:M_fd
    qk = q(k, :); qdk = qd(k, :); tauk = tau_meas(k, :);
    qdd_cad(k, :) = forward_dynamics_dispatch(qk, qdk, tauk, model_cad, struct()).';
    qdd_min(k, :) = forward_dynamics_dispatch(qk, qdk, tauk, model_min, struct()).';
    if ~isempty(pi_fd)
        qdd_fd(k, :) = forward_dynamics_dispatch(qk, qdk, tauk, model_fd, struct()).';
    end
end

qd_cad = zeros(M_fd, n); qd_cad(1,:) = qd_ref(1,:);
qd_min = zeros(M_fd, n); qd_min(1,:) = qd_ref(1,:);
qd_fd = zeros(M_fd, n); qd_fd(1,:) = qd_ref(1,:);
for k = 2:M_fd
    dt_k = t_fd(k) - t_fd(k-1);
    if dt_k <= 0 || isnan(dt_k), dt_k = median(diff(t_fd)); end
    qd_cad(k,:) = qd_cad(k-1,:) + dt_k * qdd_cad(k-1,:);
    qd_min(k,:) = qd_min(k-1,:) + dt_k * qdd_min(k-1,:);
    if ~isempty(pi_fd)
        qd_fd(k,:) = qd_fd(k-1,:) + dt_k * qdd_fd(k-1,:);
    end
end

fprintf('\n===== 正动力学 RMSE（vs qdd_ref） =====\n');
fprintf('  CAD: '); fprintf(' %.3f', sqrt(mean((qdd_cad - qdd_ref).^2, 1))); fprintf('\n');
fprintf('  MIN: '); fprintf(' %.3f', sqrt(mean((qdd_min - qdd_ref).^2, 1))); fprintf('\n');
if ~isempty(pi_fd)
    fprintf('  FD : '); fprintf(' %.3f', sqrt(mean((qdd_fd - qdd_ref).^2, 1))); fprintf('\n');
end

fprintf('\n===== 正动力学速度 RMSE（vs qd_ref） =====\n');
fprintf('  CAD: '); fprintf(' %.3f', sqrt(mean((qd_cad - qd_ref).^2, 1))); fprintf('\n');
fprintf('  MIN: '); fprintf(' %.3f', sqrt(mean((qd_min - qd_ref).^2, 1))); fprintf('\n');
if ~isempty(pi_fd)
    fprintf('  FD : '); fprintf(' %.3f', sqrt(mean((qd_fd - qd_ref).^2, 1))); fprintf('\n');
end

if ~isempty(pi_fd)
    plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
        [qd_fd, qd_cad, qd_min], {'int(FD(\pi_{fd}))', 'int(FD(CAD))', 'int(FD(\beta))'}, ...
        'qdd', '正动力学速度调试');
else
    plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
        [qd_cad, qd_min], {'int(FD(CAD))', 'int(FD(\beta))'}, ...
        'qdd', '正动力学速度调试');
end

if ~isempty(pi_fd)
    plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
        [qdd_fd, qdd_cad, qdd_min], {'FD(\pi_{fd})', 'FD(CAD)', 'FD(\beta)'}, ...
        'qdd', '正动力学调试');
else
    plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
        [qdd_cad, qdd_min], {'FD(CAD)', 'FD(\beta)'}, ...
        'qdd', '正动力学调试');
end

fprintf('\n===== 调试完成 =====\n');

