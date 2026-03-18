function avg_data = continuous_window_id_data(t, q, qd, tau, opts, qdd_given)
% continuous_window_id_data  取激励段 → 可选 qd 低通 → qdd 由 qd 差分或直接采用给定 qdd
%
% 推荐：只取中间激励段（如 t∈[3,5]s）。若已用 preprocess_id_data 得到同源 q/qd/qdd，可传入 qdd_given 跳过内部差分。
% 流程：时间窗 → 可选 qd 二阶低通 → qdd = 给定或由 qd 中心差分 → 可选 SG 平滑 qdd → 可选高|qdd|降权。
%
% 用法：
%   avg_data = continuous_window_id_data(t, q, qd, tau);
%   % 已预处理时直接传入 qdd，不再重算：
%   avg_data = continuous_window_id_data(t, q_s, qd_s, tau_id, opts, qdd_s);
%
% 可选 opts：
%   t_start_s, t_end_s - 激励段绝对时间 (s)
%   trim_start_s, trim_end_s, trim_ratio - 或按首尾裁
%   qd_lowpass_fc_Hz - qd 二阶低通截止 (Hz)，0=不滤波
%   qdd_smooth_half - 对 qdd 做 SG 平滑的半窗长，0=不平滑（若已用预处理 qdd 建议 0）
%   downweight_qdd_top_ratio - 对 |qdd| 最大的一小部分降权，默认 0.02
% 可选第六参 qdd_given - M×n，若提供则不再由 qd 差分，直接使用（与 q 同长度）

if nargin < 5
    opts = struct();
end
if nargin < 6
    qdd_given = [];
end
if ~isfield(opts, 't_start_s'),      opts.t_start_s      = []; end
if ~isfield(opts, 't_end_s'),        opts.t_end_s        = []; end
if ~isfield(opts, 'trim_start_s'),   opts.trim_start_s   = 0; end
if ~isfield(opts, 'trim_end_s'),     opts.trim_end_s     = 0; end
if ~isfield(opts, 'trim_ratio'),     opts.trim_ratio     = 0; end
if ~isfield(opts, 'qd_lowpass_fc_Hz'), opts.qd_lowpass_fc_Hz = 0; end
if ~isfield(opts, 'qdd_smooth_half'), opts.qdd_smooth_half = 0; end
if ~isfield(opts, 'downweight_qdd_top_ratio'), opts.downweight_qdd_top_ratio = 0.02; end

t   = t(:);
N   = numel(t);
n   = size(q, 2);
if size(q,1) ~= N || size(tau,1) ~= N
    error('continuous_window_id_data: t, q, tau 行数需一致。');
end
if isempty(qd) || size(qd,1) ~= N
    error('continuous_window_id_data: 需提供 qd (N×n)，用于计算 qdd。');
end

% 1) 时间窗：优先用 t_start_s / t_end_s（只取中间激励段），否则掐头去尾
if ~isempty(opts.t_start_s) || ~isempty(opts.t_end_s)
    t_start_s = opts.t_start_s;
    t_end_s   = opts.t_end_s;
    if isempty(t_start_s), t_start_s = t(1) - 1; end
    if isempty(t_end_s),   t_end_s   = t(end) + 1; end
    idx_win = t >= t_start_s & t <= t_end_s;
else
    if opts.trim_ratio > 0
        dur = t(end) - t(1);
        trim_start_s = opts.trim_ratio * dur;
        trim_end_s   = opts.trim_ratio * dur;
    else
        trim_start_s = opts.trim_start_s;
        trim_end_s   = opts.trim_end_s;
    end
    idx_win = t >= (t(1) + trim_start_s) & t <= (t(end) - trim_end_s);
end
if sum(idx_win) < 10
    warning('continuous_window_id_data: 时间窗内样本过少，改用全段。');
    idx_win = true(N, 1);
end
t_win   = t(idx_win);
q_win   = q(idx_win, :);
qd_win  = qd(idx_win, :);
tau_win = tau(idx_win, :);
M       = size(q_win, 1);
dt      = median(diff(t_win));
if dt <= 0 || isnan(dt), dt = 0.002; end
fs      = 1 / dt;

% 2) 可选：qd 二阶低通（未用预处理同源 qd 时可开；预处理后建议 0）
if opts.qd_lowpass_fc_Hz > 0 && fs > 2*opts.qd_lowpass_fc_Hz
    fc = opts.qd_lowpass_fc_Hz;
    [b, a] = butter(2, fc / (fs/2));
    for j = 1:n
        qd_win(:, j) = filtfilt(b, a, qd_win(:, j));
    end
end

% 3) qdd：若传入 qdd_given 则直接使用（与预处理同源），否则由 qd 中心差分
use_given_qdd = (nargin >= 6 && ~isempty(qdd_given) && size(qdd_given,1)==N && size(qdd_given,2)==n);
if use_given_qdd
    qdd_win = qdd_given(idx_win, :);
else
    qdd_win = zeros(M, n);
    for j = 1:n
        for k = 1:M
            if k == 1
                qdd_win(k,j) = (qd_win(2,j) - qd_win(1,j)) / dt;
            elseif k == M
                qdd_win(k,j) = (qd_win(M,j) - qd_win(M-1,j)) / dt;
            else
                qdd_win(k,j) = (qd_win(k+1,j) - qd_win(k-1,j)) / (2*dt);
            end
        end
    end
end

% 4) SG 平滑 qdd（建议，去尖峰）
if opts.qdd_smooth_half > 0
    half = min(opts.qdd_smooth_half, floor((M-1)/2));
    ord = min(3, 2*half);
    for j = 1:n
        qdd_win(:, j) = sgolayfilt(qdd_win(:, j), ord, 2*half+1);
    end
end

% 5) 降权：对 |qdd| 最大的 top 比例样本放大 tau_std（WLS 时权小）
tau_std_win = ones(M, n);
if opts.downweight_qdd_top_ratio > 0
    % 每行取 max over joints 或 2-norm 作为“该点加速度大小”
    norm_qdd = max(abs(qdd_win), [], 2);  % M×1
    n_drop = max(1, round(M * opts.downweight_qdd_top_ratio));
    [~, idx_top] = maxk(norm_qdd, n_drop);
    % 降权：tau_std 大则 identify_min 里权 w = 1/(tau_std+eps) 小
    scale = 10;
    for i = 1:numel(idx_top)
        tau_std_win(idx_top(i), :) = scale;
    end
end

% 6) 输出结构（与 cycle_average 兼容）
avg_data = struct();
avg_data.phi      = (0:(M-1))' / M;
avg_data.q_bar   = q_win;
avg_data.qd_bar  = qd_win;
avg_data.qdd_bar = qdd_win;
avg_data.tau_bar = tau_win;
avg_data.tau_std = tau_std_win;
avg_data.per_cycle = struct('q', {[]}, 'qd', {[]}, 'tau', {[]});
% 用于连续模式绘图：等效时间即真实时间
avg_data.t_equiv = t_win;
end
