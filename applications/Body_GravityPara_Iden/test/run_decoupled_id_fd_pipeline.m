%% run_decoupled_id_fd_pipeline  解耦版流程示例：数据集构建 -> 辨识分发 -> 动力学分发
%
% 目标：
%   1) 读取数据与预处理独立
%   2) 辨识算法可切换
%   3) 正逆动力学模型可切换

clc; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 构建“可直接辨识/验证”的统一数据集
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

[dataset, meta] = build_id_dataset_from_csv(csv_file, cfg); %#ok<NASGU>
fprintf('数据集构建完成: M=%d, t=[%.3f, %.3f]\n', size(dataset.q, 1), dataset.t(1), dataset.t(end));

%% 2) 辨识（方法可切换）
id_method = 'min_ls';   % 'min_ls' | 'full_fd'
id_opts = struct('limb', 'left_leg', 'para_order', 1);
id_opts.identify_min_opts = struct('use_wls', false, 'para_order', 1);
id_res = identify_params_dispatch(dataset, id_method, id_opts);
fprintf('辨识完成: method=%s\n', id_res.method);

%% 3) 逆动力学与正动力学验证（模型可切换）
k = min(50, size(dataset.q, 1));
qk = dataset.q(k, :);
qdk = dataset.qd(k, :);
qddk = dataset.qdd(k, :);
tauk = dataset.tau(k, :);

% 逆动力学：最小参数模型
id_model = struct('type', 'min', 'limb', 'left_leg', 'para_order', 1, ...
    'X_hat', id_res.X_hat, 'index_base', id_res.index_base);
tau_id = inverse_dynamics_dispatch(qk, qdk, qddk, id_model);

% 正动力学：最小参数模型
fd_model = id_model;
qdd_fd = forward_dynamics_dispatch(qk, qdk, tauk, fd_model, struct());

fprintf('单点验证 k=%d:\n', k);
fprintf('  tau_meas: '); fprintf(' %.4f', tauk); fprintf('\n');
fprintf('  tau_id(min): '); fprintf(' %.4f', tau_id); fprintf('\n');
fprintf('  qdd_ref: '); fprintf(' %.4f', qddk); fprintf('\n');
fprintf('  qdd_fd(min): '); fprintf(' %.4f', qdd_fd); fprintf('\n');

