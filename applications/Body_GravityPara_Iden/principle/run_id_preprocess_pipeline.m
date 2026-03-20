function [data_before, data_after, prep_used] = run_id_preprocess_pipeline(csv_file, prep_opts)
% run_id_preprocess_pipeline  预处理统一入口：低通滤波 + 可选补偿（解耦辨识主流程）
%
% 目标：
% - 把“滤波 + 补偿”集中在预处理函数，不混在辨识代码里。
% - 补偿可选：do_compensation=false 时仅做低通预处理，不做 J_eq/摩擦/偏置补偿。
%
% 关键选项（prep_opts）：
%   基础滤波：
%     t_start_s, t_end_s, q_lowpass_fc_Hz, q_lowpass_order,
%     tau_lowpass_fc_Hz, tau_lowpass_order, do_plot
%
%   补偿开关与来源：
%     do_compensation            - true/false，默认 false
%     J_eq, friction_params, tau_bias
%     load_friction_from_summary - true 时从 Friction_Iden 汇总表加载
%     n_joints, row_for_joint, friction_iden_dir
%
% 输出：
%   data_before: 预处理前的原始数据，结构体包含 .t, .q, .qd, .qdd, .tau
%   data_after:  预处理后的数据，结构体包含 .t, .q, .qd, .qdd, .tau_id, .tau_s
%   prep_used:   实际使用的预处理选项

if nargin < 2
    prep_opts = struct();
end

prep = prep_opts;
if ~isfield(prep, 't_start_s'), prep.t_start_s = 2.1; end
if ~isfield(prep, 't_end_s'), prep.t_end_s = 4.1; end
if ~isfield(prep, 'q_lowpass_fc_Hz'), prep.q_lowpass_fc_Hz = 25; end
if ~isfield(prep, 'q_lowpass_order'), prep.q_lowpass_order = 2; end
if ~isfield(prep, 'tau_lowpass_fc_Hz'), prep.tau_lowpass_fc_Hz = 25; end
if ~isfield(prep, 'tau_lowpass_order'), prep.tau_lowpass_order = 2; end
if ~isfield(prep, 'do_plot'), prep.do_plot = false; end

if ~isfield(prep, 'do_compensation'), prep.do_compensation = false; end
if ~isfield(prep, 'load_friction_from_summary'), prep.load_friction_from_summary = false; end
if ~isfield(prep, 'n_joints'), prep.n_joints = 6; end
if ~isfield(prep, 'row_for_joint'), prep.row_for_joint = []; end
if ~isfield(prep, 'friction_iden_dir'), prep.friction_iden_dir = ''; end

% 读取 raw
raw = read_leg_joint_csv(csv_file);
t = raw.time(:);
q = raw.pos_leg_l;
qd = raw.vel_leg_l;
tau = raw.torque_leg_l;

% 预处理前：中心差分 qdd + 时间窗裁剪（与 preprocess 一致）
N = length(t);
dt = median(diff(t));
if dt <= 0 || isnan(dt), dt = 0.002; end
qdd_before_full = zeros(N, 6);
for j = 1:6
    for k = 1:N
        qdd_before_full(k, j) = central_diff_point(qd(:, j), t, k, dt);
    end
end

idx_win = true(N, 1);
if ~isempty(prep.t_start_s), idx_win = idx_win & (t >= prep.t_start_s); end
if ~isempty(prep.t_end_s), idx_win = idx_win & (t <= prep.t_end_s); end
bad = any(~isfinite(q), 2) | any(~isfinite(tau), 2);
idx_win = idx_win & ~bad;

data_before = struct();
data_before.t = t(idx_win);
data_before.q = q(idx_win, :);
data_before.qd = qd(idx_win, :);
data_before.qdd = qdd_before_full(idx_win, :);
data_before.tau = tau(idx_win, :);

% 预处理后
if prep.do_compensation
    if prep.load_friction_from_summary
        friction_iden_dir = prep.friction_iden_dir;
        if isempty(friction_iden_dir)
            app_root = fileparts(fileparts(mfilename('fullpath')));
            friction_iden_dir = fullfile(app_root, '..', 'Friction_Iden');
        end
        if isfolder(friction_iden_dir) && isempty(which('load_friction_stribeck_from_summary'))
            addpath(friction_iden_dir);
        end
        load_opts = struct();
        if ~isempty(prep.row_for_joint), load_opts.row_for_joint = prep.row_for_joint; end
        [fp_loaded, Ia_loaded] = load_friction_stribeck_from_summary(prep.n_joints, load_opts);
        if ~isfield(prep, 'friction_params') || isempty(prep.friction_params)
            prep.friction_params = fp_loaded;
        end
        if ~isfield(prep, 'J_eq') || isempty(prep.J_eq)
            prep.J_eq = Ia_loaded;
        end
    end
else
    % 禁用补偿时，强制清空补偿项，确保只做滤波
    prep.J_eq = [];
    prep.friction_params = [];
    prep.tau_bias = [];
end

[t_after, q_after, qd_after, qdd_after, tau_id_after, aux] = preprocess_id_data(t, q, qd, tau, prep);

data_after = struct();
data_after.t = t_after;
data_after.q = q_after;
data_after.qd = qd_after;
data_after.qdd = qdd_after;
data_after.tau_id = tau_id_after;
data_after.tau_s = aux.tau_s;

prep_used = prep;

end