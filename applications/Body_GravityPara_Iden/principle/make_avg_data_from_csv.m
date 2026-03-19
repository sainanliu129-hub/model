function [avg_data, meta] = make_avg_data_from_csv(csv_file, cfg)
% make_avg_data_from_csv  CSV -> (可选低通预处理) -> 连续窗 -> avg_data
%
% 说明：
% - 已统一为低通预处理链路，不再支持 cycle_average。
% - 预处理默认参数与 plot_preprocess_before_after_trajectories 一致：
%   t=[2.1,4.1], q_lowpass_fc_Hz=25, q_lowpass_order=2,
%   tau_lowpass_fc_Hz=25, tau_lowpass_order=2。

if nargin < 2, cfg = struct(); end
if nargin < 1 || isempty(csv_file)
    error('make_avg_data_from_csv: csv_file 不能为空。');
end

cfg = set_default(cfg, 'limb', 'left_leg');
cfg = set_default(cfg, 'use_preprocess', true);
cfg = set_default(cfg, 'prep_opts', struct());
cfg = set_default(cfg, 'window_opts', struct());

data = read_leg_joint_csv(csv_file);
t = data.time(:);
if strcmpi(cfg.limb, 'left_leg')
    q = data.pos_leg_l;
    qd = data.vel_leg_l;
    tau = data.torque_leg_l;
else
    error('make_avg_data_from_csv: 目前仅实现 left_leg。');
end

qdd_for_win = [];
prep_opts_used = [];
prep_aux = struct();
if cfg.use_preprocess
    prep = cfg.prep_opts;
    prep = set_default(prep, 't_start_s', 2.1);
    prep = set_default(prep, 't_end_s', 4.1);
    prep = set_default(prep, 'q_lowpass_fc_Hz', 25);
    prep = set_default(prep, 'q_lowpass_order', 2);
    prep = set_default(prep, 'tau_lowpass_fc_Hz', 25);
    prep = set_default(prep, 'tau_lowpass_order', 2);
    prep = set_default(prep, 'do_plot', false);
    [t, q, qd, qdd_for_win, tau, prep_aux] = preprocess_id_data(t, q, qd, tau, prep);
    prep_opts_used = prep;
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
avg_data = continuous_window_id_data(t, q, qd, tau, win, qdd_for_win);

meta = struct();
cfg.mode = 'continuous';
meta.csv_file = csv_file;
meta.cfg = cfg;
meta.raw = struct('t', t, 'q', q, 'qd', qd, 'tau', tau);
meta.qdd_for_win = qdd_for_win;
meta.prep_opts_used = prep_opts_used;
meta.prep_aux = prep_aux;
end

function s = set_default(s, name, val)
if ~isfield(s, name) || isempty(s.(name))
    s.(name) = val;
end
end

