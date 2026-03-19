function run_torque_comparison(csv_file, opts_in)
% run_torque_comparison  解耦版力矩对比：tau_meas vs NE刚体 vs NE+补偿 vs 最小参数辨识

clc; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

if nargin < 2, opts_in = struct(); end
if nargin < 1 || isempty(csv_file)
    csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
end
if ischar(csv_file) && exist(csv_file, 'file') ~= 2
    csv_file = fullfile(app_root, csv_file);
end
if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

% 配置
t_start_s = get_field_or(opts_in, 't_start_s', 2.1);
t_end_s = get_field_or(opts_in, 't_end_s', 4.1);
use_saved_id = get_field_or(opts_in, 'use_saved_id', false);
save_excel = get_field_or(opts_in, 'save_excel', true);

cfg = struct();
cfg.use_preprocess = true;
cfg.prep_opts = struct( ...
    't_start_s', t_start_s, ...
    't_end_s', t_end_s, ...
    'q_lowpass_fc_Hz', 25, ...
    'q_lowpass_order', 2, ...
    'tau_lowpass_fc_Hz', 25, ...
    'tau_lowpass_order', 2, ...
    'do_compensation', true, ...
    'load_friction_from_summary', true, ...
    'do_plot', false);
cfg.window_opts = struct('t_start_s', [], 't_end_s', [], 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

[dataset, meta] = build_id_dataset_from_csv(csv_file, cfg);
M = size(dataset.q, 1);
limb = 'left_leg';
para_order = 1;

% 辨识结果（加载或重算）
out_mat = fullfile(app_root, 'min_param_id_result.mat');
if use_saved_id && isfile(out_mat)
    ld = load(out_mat);
    X_hat = ld.X_hat; index_base = ld.index_base; metrics = ld.metrics;
else
    id_opts = struct('limb', limb, 'para_order', para_order, ...
        'identify_min_opts', struct('use_wls', false, 'para_order', para_order));
    res_min = identify_params_dispatch(dataset, 'min_ls', id_opts);
    X_hat = res_min.X_hat; index_base = res_min.index_base; metrics = res_min.metrics;
end

% 模型
model_ne = struct('type', 'urdf', 'limb', limb, 'para_order', para_order);
model_min = struct('type', 'min', 'limb', limb, 'para_order', para_order, ...
    'X_hat', X_hat, 'index_base', index_base);

tau_meas = dataset.tau;
tau_ne = zeros(M, 6);
tau_pred_id = metrics.tau_pred;
for k = 1:M
    tau_ne(k, :) = inverse_dynamics_dispatch(dataset.q(k,:), dataset.qd(k,:), dataset.qdd(k,:), model_ne).';
end

tau_comp = zeros(M, 6);
if isfield(meta, 'prep_aux') && isfield(meta.prep_aux, 'tau_comp') && ~isempty(meta.prep_aux.tau_comp)
    tau_comp = meta.prep_aux.tau_comp(1:M, :);
end
tau_ne_plus = tau_ne + tau_comp;

% 误差
err_ne = tau_ne - tau_meas;
err_ne_plus = tau_ne_plus - tau_meas;
err_id = tau_pred_id - tau_meas;

fprintf('RMSE (N·m):\n');
fprintf('  NE_rigid   :'); fprintf(' %.3f', sqrt(mean(err_ne.^2,1))); fprintf('\n');
fprintf('  NE+prepComp:'); fprintf(' %.3f', sqrt(mean(err_ne_plus.^2,1))); fprintf('\n');
fprintf('  id_pred    :'); fprintf(' %.3f', sqrt(mean(err_id.^2,1))); fprintf('\n');

t = dataset.t;
plot_compare_with_error_6dof(t, tau_meas, 'tau\_meas', ...
    [tau_ne, tau_ne_plus, tau_pred_id], ...
    {'tau\_NE\_rigid', 'tau\_NE+prep\_comp', 'tau\_id\_pred'}, ...
    'torque', '力矩多版本对比（解耦版）');

if save_excel
    id_rmse = struct();
    id_rmse.ne_rigid = sqrt(mean(err_ne.^2, 1));
    id_rmse.ne_plus_comp = sqrt(mean(err_ne_plus.^2, 1));
    id_rmse.id_pred = sqrt(mean(err_id.^2, 1));
    out_xlsx = fullfile(app_root, 'build', 'torque_comparison_summary.xlsx');
    meta_tbl = table(string(csv_file), size(dataset.q,1), ...
        'VariableNames', {'csv_file','num_samples'});
    export_validation_summary_excel(out_xlsx, id_rmse, struct(), meta_tbl);
end
end

function v = get_field_or(s, name, default_v)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_v;
end
end

