function [dataset, meta] = build_id_dataset_from_csv(csv_file, cfg)
% build_id_dataset_from_csv  读取CSV并构建可直接用于辨识/动力学验证的数据集
%
% 解耦目标：
%   read csv -> preprocess(filter + optional compensation) -> continuous window
%   输出统一 dataset，供 identify/FD/ID 复用，避免重复处理。
%
% cfg 可选字段：
%   .limb              默认 'left_leg'
%   .use_preprocess    默认 true
%   .prep_opts         传给 run_id_preprocess_pipeline
%   .window_opts       传给 continuous_window_id_data
%
% 输出 dataset：
%   .t, .q, .qd, .qdd, .tau      -> 直接可用于辨识与正逆动力学验证
%   .avg_data                    -> continuous_window_id_data 原始输出

if nargin < 2, cfg = struct(); end
if nargin < 1 || isempty(csv_file)
    error('build_id_dataset_from_csv: csv_file 不能为空。');
end

cfg = set_default(cfg, 'limb', 'left_leg');
cfg = set_default(cfg, 'use_preprocess', true);
cfg = set_default(cfg, 'prep_opts', struct());
cfg = set_default(cfg, 'window_opts', struct());

raw = read_leg_joint_csv(csv_file);
t = raw.time(:);
if strcmpi(cfg.limb, 'left_leg')
    q = raw.pos_leg_l;
    qd = raw.vel_leg_l;
    tau = raw.torque_leg_l;
else
    error('build_id_dataset_from_csv: 目前仅支持 left_leg。');
end

prep_aux = struct();
prep_used = struct();
qdd_given = [];
if cfg.use_preprocess
    [t, q, qd, qdd_given, tau, prep_aux, prep_used] = run_id_preprocess_pipeline(t, q, qd, tau, cfg.prep_opts);
end

win = cfg.window_opts;
if cfg.use_preprocess
    win = set_default(win, 't_start_s', []);
    win = set_default(win, 't_end_s', []);
    win = set_default(win, 'qd_lowpass_fc_Hz', 0);
    win = set_default(win, 'qdd_smooth_half', 0);
else
    win = set_default(win, 't_start_s', 2.1);
    win = set_default(win, 't_end_s', 4.1);
    win = set_default(win, 'qd_lowpass_fc_Hz', 0);
    win = set_default(win, 'qdd_smooth_half', 0);
end
avg_data = continuous_window_id_data(t, q, qd, tau, win, qdd_given);

dataset = struct();
dataset.t = avg_data.t_equiv(:);
dataset.q = avg_data.q_bar;
dataset.qd = avg_data.qd_bar;
dataset.qdd = avg_data.qdd_bar;
dataset.tau = avg_data.tau_bar;
dataset.avg_data = avg_data;

meta = struct();
meta.csv_file = csv_file;
meta.cfg = cfg;
meta.prep_used = prep_used;
meta.prep_aux = prep_aux;
meta.window_used = win;
end

function s = set_default(s, name, val)
if ~isfield(s, name) || isempty(s.(name))
    s.(name) = val;
end
end

