function [avg_data, meta] = make_avg_data_from_csv(csv_file, cfg)
% make_avg_data_from_csv  CSV -> (可选预处理) -> 连续窗/周期平均 -> avg_data
%
% 目的：把“预处理/取窗”独立成函数，供辨识与验证脚本复用。
%
% 输入：
%   csv_file : CSV 路径（与 read_leg_joint_csv 兼容）
%   cfg      : 可选结构体（字段缺省则用默认）：
%     .limb              默认 'left_leg'（目前仅用于选择左腿数据列）
%     .mode              'continuous'(默认) | 'cycle_average'
%
%     % 预处理（preprocess_id_data）
%     .use_preprocess    默认 true
%     .prep_opts         传给 preprocess_id_data 的 opts（如 t_start_s/t_end_s/sg_order/sg_frame/tau_lowpass...）
%
%     % 连续窗（continuous_window_id_data）
%     .window_opts       传给 continuous_window_id_data 的 opts（如 t_start_s/t_end_s/qd_lowpass_fc_Hz/qdd_smooth_half）
%
%     % 周期平均（cycle_average）
%     .K                 周期数（默认 5）
%     .T                 单周期时长（默认 8）
%     .trim_ramp         默认 1.0
%     .anchor_joint      默认 0（自动）
%     .anchor_type       默认 'fixed_period'
%     .ref_t/.ref_q/.ref_qd  可选相位网格
%
% 输出：
%   avg_data : 与 identify_min/后续脚本兼容的结构体（q_bar/qd_bar/qdd_bar/tau_bar/tau_std/t_equiv）
%   meta     : 辅助信息结构体：raw/prep_opts_used/prep_aux 等

    if nargin < 2, cfg = struct(); end
    if nargin < 1 || isempty(csv_file)
        error('make_avg_data_from_csv: csv_file 不能为空。');
    end

    cfg = set_default(cfg, 'limb', 'left_leg');
    cfg = set_default(cfg, 'mode', 'continuous');
    cfg = set_default(cfg, 'use_preprocess', true);
    cfg = set_default(cfg, 'prep_opts', struct());
    cfg = set_default(cfg, 'window_opts', struct());

    cfg = set_default(cfg, 'K', 5);
    cfg = set_default(cfg, 'T', 8);
    cfg = set_default(cfg, 'trim_ramp', 1.0);
    cfg = set_default(cfg, 'anchor_joint', 0);
    cfg = set_default(cfg, 'anchor_type', 'fixed_period');
    cfg = set_default(cfg, 'ref_t', []);
    cfg = set_default(cfg, 'ref_q', []);
    cfg = set_default(cfg, 'ref_qd', []);

    % 1) 读 CSV
    data = read_leg_joint_csv(csv_file);
    t = data.time(:);
    if strcmpi(cfg.limb, 'left_leg')
        q   = data.pos_leg_l;
        qd  = data.vel_leg_l;
        tau = data.torque_leg_l;
    else
        error('make_avg_data_from_csv: 目前仅实现 left_leg（可按需要扩展 right_leg/arm）。');
    end

    % 2) 可选预处理（推荐）
    qdd_for_win = [];
    prep_opts_used = [];
    prep_aux = struct();
    if cfg.use_preprocess
        prep = cfg.prep_opts;
        prep = set_default(prep, 't_start_s', 2.1);
        prep = set_default(prep, 't_end_s', 4.1);
        prep = set_default(prep, 'sg_order', 4);
        prep = set_default(prep, 'sg_frame', 23);
        prep = set_default(prep, 'tau_lowpass_fc_Hz', 15);
        prep = set_default(prep, 'tau_lowpass_order', 2);
        prep = set_default(prep, 'do_plot', false);
        [t, q, qd, qdd_for_win, tau, prep_aux] = preprocess_id_data(t, q, qd, tau, prep);
        prep_opts_used = prep;
    end

    % 3) 连续窗 / 周期平均 -> avg_data
    if strcmpi(cfg.mode, 'continuous')
        win = cfg.window_opts;
        if cfg.use_preprocess
            % 预处理已做时间窗与同源 q/qd/qdd：这里默认不再滤波、不再重算 qdd
            win = set_default(win, 't_start_s', []);
            win = set_default(win, 't_end_s', []);
            win = set_default(win, 'qd_lowpass_fc_Hz', 0);
            win = set_default(win, 'qdd_smooth_half', 0);
        else
            win = set_default(win, 't_start_s', 2.1);
            win = set_default(win, 't_end_s', 4.1);
            win = set_default(win, 'qd_lowpass_fc_Hz', 50);
            win = set_default(win, 'qdd_smooth_half', 15);
        end
        avg_data = continuous_window_id_data(t, q, qd, tau, win, qdd_for_win);
    elseif strcmpi(cfg.mode, 'cycle_average')
        ca_args = {'qd', qd, 'trim_ramp', cfg.trim_ramp, 'anchor_joint', cfg.anchor_joint, 'anchor_type', cfg.anchor_type};
        if ~isempty(cfg.ref_t) && ~isempty(cfg.ref_q)
            ca_args = [ca_args, {'ref_t', cfg.ref_t, 'ref_q', cfg.ref_q}];
            if ~isempty(cfg.ref_qd)
                ca_args = [ca_args, {'ref_qd', cfg.ref_qd}];
            end
        end
        avg_data = cycle_average(t, q, tau, cfg.K, cfg.T, ca_args{:});
        avg_data.t_equiv = avg_data.phi * cfg.T;
        % cycle_average 不提供 qdd：这里补一个简单中心差分（与其它脚本一致性要求不高时可用）
        if ~isfield(avg_data, 'qdd_bar') || isempty(avg_data.qdd_bar)
            qdd_bar = zeros(size(avg_data.qd_bar));
            dt = median(diff(avg_data.t_equiv));
            if dt <= 0 || isnan(dt), dt = 0.002; end
            for j = 1:size(qdd_bar,2)
                for k = 1:size(qdd_bar,1)
                    qdd_bar(k,j) = central_diff_point(avg_data.qd_bar(:,j), avg_data.t_equiv, k, dt);
                end
            end
            avg_data.qdd_bar = qdd_bar;
        end
        if ~isfield(avg_data, 'tau_std') || isempty(avg_data.tau_std)
            avg_data.tau_std = ones(size(avg_data.tau_bar));
        end
    else
        error('make_avg_data_from_csv: 未知 cfg.mode=%s（应为 continuous/cycle_average）。', cfg.mode);
    end

    meta = struct();
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

