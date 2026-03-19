%% run_full_dynamics_validation  解耦版全流程验证
% 流程：构建数据集 -> 最小参数辨识 -> full参数恢复/辨识 -> ID/FD 多模型对比

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 配置
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');
if ~isfile(csv_file)
    error('CSV 不存在: %s', csv_file);
end

cfg = struct();
cfg.use_preprocess = true;
cfg.prep_opts = struct( ...
    't_start_s', 2.1, ...
    't_end_s', 4.1, ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', false, ...
    'do_plot', false);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

limb = 'left_leg';
para_order = 1;
run_pi_phys = true;
run_pi_fd = true;
M_fd_max = inf;
save_excel = true;

%% 2) 构建统一数据集（读取+预处理+连续窗）
[dataset, meta] = build_id_dataset_from_csv(csv_file, cfg); %#ok<NASGU>
fprintf('数据集: M=%d, t=[%.3f, %.3f]\n', size(dataset.q,1), dataset.t(1), dataset.t(end));

%% 3) 最小参数辨识
id_opts = struct();
id_opts.limb = limb;
id_opts.para_order = para_order;
id_opts.identify_min_opts = struct('use_wls', false, 'para_order', para_order);
res_min = identify_params_dispatch(dataset, 'min_ls', id_opts);
X_hat = res_min.X_hat;
index_base = res_min.index_base;
fprintf('最小参数维度 p_min=%d\n', numel(X_hat));

%% 4) CAD/full 参数
[robot_limb, n] = get_e1_limb_robot(limb); %#ok<ASGLU>
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
Y_full = ReMatrix_E1_limb_URDF(limb, dataset.q, dataset.qd, dataset.qdd, 1, para_order);
Y_min = Y_full(:, index_base);
K = pinv(Y_min) * Y_full;
beta_hat = X_hat(:);
[pi_rec, res_norm_rec] = recover_full_params_from_beta(K, beta_hat, pi_cad, 1e-2);
fprintf('pi_rec 恢复残差=%.3e\n', res_norm_rec);

pi_phys = [];
if run_pi_phys
    opts_phys = struct('algorithm','sqp','max_iter',300,'display','iter');
    [pi_phys, info_phys] = solve_full_params_physical(Y_full, Y_min, beta_hat, pi_cad, 10, opts_phys); %#ok<NASGU>
end

pi_fd = [];
if run_pi_fd
    opts_full = struct();
    opts_full.limb = limb;
    opts_full.para_order = para_order;
    opts_full.pi_cad = pi_cad;
    opts_full.identify_full_fd_opts = struct('w_tau',1,'w_qdd',10,'w_cad',1,'w_M',10, ...
        'max_iter',30,'MaxFunctionEvaluations',600,'display','iter');
    res_full = identify_params_dispatch(dataset, 'full_fd', opts_full);
    pi_fd = res_full.pi_fd;
end

%% 5) 逆动力学对比
M = size(dataset.q, 1);
tau_meas = dataset.tau;
tau_cad = zeros(M, n);
tau_min = zeros(M, n);
tau_rec = zeros(M, n);
tau_phys = zeros(M, n);
tau_fd = zeros(M, n);

model_cad_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_cad);
model_min_id = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
model_rec_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
if ~isempty(pi_phys), model_phys_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
if ~isempty(pi_fd), model_fd_id = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

for k = 1:M
    tau_cad(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_cad_id).';
    tau_min(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_min_id).';
    tau_rec(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_rec_id).';
    if ~isempty(pi_phys), tau_phys(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_phys_id).'; end
    if ~isempty(pi_fd), tau_fd(k,:) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_fd_id).'; end
end

fprintf('\nID RMSE (vs tau_meas):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((tau_cad-tau_meas).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((tau_min-tau_meas).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((tau_rec-tau_meas).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((tau_phys-tau_meas).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((tau_fd-tau_meas).^2,1))); fprintf('\n'); end

plot_data = [tau_meas, tau_cad, tau_min, tau_rec];
legend_txt = {'\tau_{meas}','Y\pi_{cad}','Y_{min}X_{hat}','Y\pi_{rec}'};
if ~isempty(pi_phys), plot_data = [plot_data, tau_phys]; legend_txt{end+1} = 'Y\pi_{phys}'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data = [plot_data, tau_fd]; legend_txt{end+1} = 'Y\pi_{fd}'; end %#ok<AGROW>
plot_compare_with_error_6dof((1:M)', tau_meas, '\tau_{meas}', ...
    plot_data(:, 7:end), legend_txt(2:end), 'torque', '逆动力学多模型');

%% 6) 正动力学对比
M_fd = min(M, M_fd_max);
t_fd = dataset.t(1:M_fd);
qd_ref = dataset.qd(1:M_fd,:);
qdd_ref = dataset.qdd(1:M_fd,:);
qdd_cad = zeros(M_fd,n);
qdd_min = zeros(M_fd,n);
qdd_rec = zeros(M_fd,n);
qdd_phys = zeros(M_fd,n);
qdd_fd = zeros(M_fd,n);

model_cad_fd = struct('type','urdf','limb',limb,'para_order',para_order);
model_min_fd = struct('type','min','limb',limb,'para_order',para_order,'X_hat',X_hat,'index_base',index_base);
model_rec_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_rec);
if ~isempty(pi_phys), model_phys_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_phys); end
if ~isempty(pi_fd), model_fd_fd = struct('type','full','limb',limb,'para_order',para_order,'pi_vec',pi_fd); end

for k = 1:M_fd
    qk = dataset.q(k,:); qdk = dataset.qd(k,:); tauk = dataset.tau(k,:);
    qdd_cad(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_cad_fd, struct()).';
    qdd_min(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_min_fd, struct()).';
    qdd_rec(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_rec_fd, struct()).';
    if ~isempty(pi_phys), qdd_phys(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_phys_fd, struct()).'; end
    if ~isempty(pi_fd), qdd_fd(k,:) = forward_dynamics_dispatch(qk, qdk, tauk, model_fd_fd, struct()).'; end
end

% 由 qdd 积分得到 qd（用于正动力学速度对比）
qd_cad = zeros(M_fd, n); qd_cad(1,:) = qd_ref(1,:);
qd_min = zeros(M_fd, n); qd_min(1,:) = qd_ref(1,:);
qd_rec = zeros(M_fd, n); qd_rec(1,:) = qd_ref(1,:);
qd_phys = zeros(M_fd, n); qd_phys(1,:) = qd_ref(1,:);
qd_fd = zeros(M_fd, n); qd_fd(1,:) = qd_ref(1,:);
for k = 2:M_fd
    dt_k = t_fd(k) - t_fd(k-1);
    if dt_k <= 0 || isnan(dt_k), dt_k = median(diff(t_fd)); end
    qd_cad(k,:) = qd_cad(k-1,:) + dt_k * qdd_cad(k-1,:);
    qd_min(k,:) = qd_min(k-1,:) + dt_k * qdd_min(k-1,:);
    qd_rec(k,:) = qd_rec(k-1,:) + dt_k * qdd_rec(k-1,:);
    if ~isempty(pi_phys), qd_phys(k,:) = qd_phys(k-1,:) + dt_k * qdd_phys(k-1,:); end
    if ~isempty(pi_fd), qd_fd(k,:) = qd_fd(k-1,:) + dt_k * qdd_fd(k-1,:); end
end

fprintf('\nFD RMSE (vs qdd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qdd_cad-qdd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qdd_min-qdd_ref).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qdd_rec-qdd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qdd_phys-qdd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qdd_fd-qdd_ref).^2,1))); fprintf('\n'); end

fprintf('\nFD 速度RMSE (vs qd_ref):\n');
fprintf('  CAD :'); fprintf(' %.3f', sqrt(mean((qd_cad-qd_ref).^2,1))); fprintf('\n');
fprintf('  MIN :'); fprintf(' %.3f', sqrt(mean((qd_min-qd_ref).^2,1))); fprintf('\n');
fprintf('  REC :'); fprintf(' %.3f', sqrt(mean((qd_rec-qd_ref).^2,1))); fprintf('\n');
if ~isempty(pi_phys), fprintf('  PHYS:'); fprintf(' %.3f', sqrt(mean((qd_phys-qd_ref).^2,1))); fprintf('\n'); end
if ~isempty(pi_fd), fprintf('  FD  :'); fprintf(' %.3f', sqrt(mean((qd_fd-qd_ref).^2,1))); fprintf('\n'); end

plot_data_qd = [qd_ref, qd_cad, qd_min, qd_rec];
legend_txt_qd = {'qd_{ref}','int(FD(CAD))','int(FD(\beta))','int(FD(\pi_{rec}))'};
if ~isempty(pi_phys), plot_data_qd = [plot_data_qd, qd_phys]; legend_txt_qd{end+1} = 'int(FD(\pi_{phys}))'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data_qd = [plot_data_qd, qd_fd]; legend_txt_qd{end+1} = 'int(FD(\pi_{fd}))'; end %#ok<AGROW>
plot_compare_with_error_6dof(t_fd, qd_ref, 'qd_{ref}', ...
    plot_data_qd(:, 7:end), legend_txt_qd(2:end), 'qdd', '正动力学速度多模型');

plot_data = [qdd_ref, qdd_cad, qdd_min, qdd_rec];
legend_txt = {'qdd_{ref}','FD(CAD)','FD(\beta)','FD(\pi_{rec})'};
if ~isempty(pi_phys), plot_data = [plot_data, qdd_phys]; legend_txt{end+1} = 'FD(\pi_{phys})'; end %#ok<AGROW>
if ~isempty(pi_fd), plot_data = [plot_data, qdd_fd]; legend_txt{end+1} = 'FD(\pi_{fd})'; end %#ok<AGROW>
plot_compare_with_error_6dof(t_fd, qdd_ref, 'qdd_{ref}', ...
    plot_data(:, 7:end), legend_txt(2:end), 'qdd', '正动力学多模型');

if save_excel
    id_rmse = struct();
    id_rmse.cad = sqrt(mean((tau_cad - tau_meas).^2,1));
    id_rmse.min = sqrt(mean((tau_min - tau_meas).^2,1));
    id_rmse.rec = sqrt(mean((tau_rec - tau_meas).^2,1));
    if ~isempty(pi_phys), id_rmse.phys = sqrt(mean((tau_phys - tau_meas).^2,1)); end
    if ~isempty(pi_fd), id_rmse.fd = sqrt(mean((tau_fd - tau_meas).^2,1)); end

    fd_rmse = struct();
    fd_rmse.qd_cad = sqrt(mean((qd_cad - qd_ref).^2,1));
    fd_rmse.qd_min = sqrt(mean((qd_min - qd_ref).^2,1));
    fd_rmse.qd_rec = sqrt(mean((qd_rec - qd_ref).^2,1));
    if ~isempty(pi_phys), fd_rmse.qd_phys = sqrt(mean((qd_phys - qd_ref).^2,1)); end
    if ~isempty(pi_fd), fd_rmse.qd_fd = sqrt(mean((qd_fd - qd_ref).^2,1)); end
    fd_rmse.cad = sqrt(mean((qdd_cad - qdd_ref).^2,1));
    fd_rmse.min = sqrt(mean((qdd_min - qdd_ref).^2,1));
    fd_rmse.rec = sqrt(mean((qdd_rec - qdd_ref).^2,1));
    if ~isempty(pi_phys), fd_rmse.phys = sqrt(mean((qdd_phys - qdd_ref).^2,1)); end
    if ~isempty(pi_fd), fd_rmse.fd = sqrt(mean((qdd_fd - qdd_ref).^2,1)); end

    meta_tbl = table(string(csv_file), size(dataset.q,1), M_fd, ...
        'VariableNames', {'csv_file','num_samples','num_fd_samples'});
    out_xlsx = fullfile(app_root, 'build', 'full_dynamics_validation_summary.xlsx');
    export_validation_summary_excel(out_xlsx, id_rmse, fd_rmse, meta_tbl);
end

%% 7) 保存输出（兼容后续脚本）
out_mat = fullfile(app_root, 'min_param_id_result.mat');
prep_opts = meta.prep_used;
avg_data = dataset.avg_data; %#ok<NASGU>
metrics = res_min.metrics; %#ok<NASGU>
save(out_mat, 'X_hat', 'index_base', 'avg_data', 'metrics', 'prep_opts');
fprintf('\n已保存: %s\n', out_mat);
fprintf('===== 全流程对比完成 =====\n');

