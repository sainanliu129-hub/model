function [t_out, q_s, qd_s, qdd_s, tau_id, aux] = preprocess_lowpass_id_data(t, q, qd_raw, tau_raw, opts)
% preprocess_lowpass_id_data  低通预处理：q低通+差分 -> tau低通 -> 裁剪 -> 构造tau_id
%
% 统一链路：
%   1) 全段 q 做 Butterworth + filtfilt（零相位）
%   2) 由 q_s 中心差分得到 qd_s, qdd_s
%   3) tau 做 Butterworth + filtfilt（零相位）
%   4) 时间裁剪/异常行剔除
%   5) tau_id = tau_s - J_eq.*qdd_s - tau_f(qd_s) - tau_bias

if nargin < 5
    opts = struct();
end
t = t(:);
N = numel(t);
n = size(q, 2);
if size(q, 1) ~= N || size(tau_raw, 1) ~= N
    error('preprocess_lowpass_id_data: t, q, tau_raw 行数须一致。');
end
if isempty(qd_raw), qd_raw = zeros(N, n); end
if size(qd_raw, 1) ~= N || size(qd_raw, 2) ~= n
    error('preprocess_lowpass_id_data: qd_raw 须为 N×n。');
end

if ~isfield(opts, 't_start_s'),       opts.t_start_s = 2.1; end
if ~isfield(opts, 't_end_s'),         opts.t_end_s = 4.1; end
if ~isfield(opts, 'trim_start_s'),    opts.trim_start_s = 0; end
if ~isfield(opts, 'trim_end_s'),      opts.trim_end_s = 0; end
if ~isfield(opts, 'trim_start_n'),    opts.trim_start_n = 0; end
if ~isfield(opts, 'trim_end_n'),      opts.trim_end_n = 0; end
if ~isfield(opts, 'remove_nan_inf'),  opts.remove_nan_inf = true; end
if ~isfield(opts, 'max_abs_q'),       opts.max_abs_q = []; end
if ~isfield(opts, 'max_abs_qd'),      opts.max_abs_qd = []; end
if ~isfield(opts, 'max_abs_tau'),     opts.max_abs_tau = []; end
if ~isfield(opts, 'q_lowpass_fc_Hz'), opts.q_lowpass_fc_Hz = 25; end
if ~isfield(opts, 'q_lowpass_order'), opts.q_lowpass_order = 2; end
if ~isfield(opts, 'Fs'),              opts.Fs = []; end
if ~isfield(opts, 'tau_lowpass_fc_Hz'), opts.tau_lowpass_fc_Hz = 25; end
if ~isfield(opts, 'tau_lowpass_order'), opts.tau_lowpass_order = 2; end
if ~isfield(opts, 'J_eq'),            opts.J_eq = []; end
if ~isfield(opts, 'friction_params'), opts.friction_params = []; end
if ~isfield(opts, 'tau_bias'),        opts.tau_bias = []; end
if ~isfield(opts, 'do_plot'),         opts.do_plot = false; end
if ~isfield(opts, 'plot_joints'),     opts.plot_joints = [1 2 3]; end

dt_full = median(diff(t));
if dt_full <= 0 || isnan(dt_full), dt_full = 0.002; end
Fs_full = opts.Fs;
if isempty(Fs_full), Fs_full = 1 / dt_full; end

q_s_full = zeros(N, n);
q_fc = opts.q_lowpass_fc_Hz;
q_ord = opts.q_lowpass_order;
if isscalar(q_fc)
    if q_fc > 0 && Fs_full > 2 * q_fc
        [bq, aq] = butter(q_ord, q_fc / (Fs_full / 2));
        for j = 1:n
            q_s_full(:, j) = filtfilt(bq, aq, q(:, j));
        end
    else
        q_s_full = q;
    end
else
    q_fc_vec = q_fc(:).';
    if numel(q_fc_vec) ~= n
        error('preprocess_lowpass_id_data: q_lowpass_fc_Hz 为向量时，长度需等于关节数 n=%d；当前=%d。', n, numel(q_fc_vec));
    end
    for j = 1:n
        fcj = q_fc_vec(j);
        if fcj > 0 && Fs_full > 2 * fcj
            [bq, aq] = butter(q_ord, fcj / (Fs_full / 2));
            q_s_full(:, j) = filtfilt(bq, aq, q(:, j));
        else
            q_s_full(:, j) = q(:, j);
        end
    end
end

qd_s_full = zeros(N, n);
qdd_s_full = zeros(N, n);
for j = 1:n
    qd_s_full(:, j) = local_center_diff(q_s_full(:, j), dt_full);
    qdd_s_full(:, j) = local_center_diff(qd_s_full(:, j), dt_full);
end

fc = opts.tau_lowpass_fc_Hz;
ord_tau = opts.tau_lowpass_order;
tau_s_full = zeros(N, n);
if fc > 0 && Fs_full > 2 * fc
    [b, a] = butter(ord_tau, fc / (Fs_full / 2));
    for j = 1:n
        tau_s_full(:, j) = filtfilt(b, a, tau_raw(:, j));
    end
else
    tau_s_full = tau_raw;
end

idx = true(N, 1);
if ~isempty(opts.t_start_s), idx = idx & (t >= opts.t_start_s); end
if ~isempty(opts.t_end_s),   idx = idx & (t <= opts.t_end_s); end
if opts.trim_start_s > 0
    t0 = t(find(idx, 1, 'first'));
    idx = idx & (t >= t0 + opts.trim_start_s);
end
if opts.trim_end_s > 0
    t1 = t(find(idx, 1, 'last'));
    idx = idx & (t <= t1 - opts.trim_end_s);
end
if opts.trim_start_n > 0
    ii = find(idx);
    if numel(ii) > opts.trim_start_n
        idx(ii(1:opts.trim_start_n)) = false;
    end
end
if opts.trim_end_n > 0
    ii = find(idx);
    if numel(ii) > opts.trim_end_n
        idx(ii(end - opts.trim_end_n + 1 : end)) = false;
    end
end
if opts.remove_nan_inf
    bad = any(~isfinite(q), 2) | any(~isfinite(tau_raw), 2);
    idx = idx & ~bad;
end
if ~isempty(opts.max_abs_q)
    thr = opts.max_abs_q(:)';
    if isscalar(thr), thr = repmat(thr, 1, n); end
    idx = idx & all(abs(q) <= thr, 2);
end
if ~isempty(opts.max_abs_qd) && any(isfinite(qd_raw(:)))
    thr = opts.max_abs_qd(:)';
    if isscalar(thr), thr = repmat(thr, 1, n); end
    idx = idx & all(abs(qd_raw) <= thr, 2);
end
if ~isempty(opts.max_abs_tau)
    thr = opts.max_abs_tau(:)';
    if isscalar(thr), thr = repmat(thr, 1, n); end
    idx = idx & all(abs(tau_raw) <= thr, 2);
end

t_out = t(idx);
q_win = q(idx, :);
tau_win = tau_raw(idx, :);
tau_s = tau_s_full(idx, :);
q_s = q_s_full(idx, :);
qd_s = qd_s_full(idx, :);
qdd_s = qdd_s_full(idx, :);
M = size(q_s, 1);

tau_Jm = zeros(M, n);
tau_f = zeros(M, n);
tau_bias_vec = zeros(M, n);
J_eq = opts.J_eq;
if ~isempty(J_eq)
    J_eq = J_eq(:)';
    if isscalar(J_eq)
        J_eq = repmat(J_eq, 1, n);
    else
        J_eq = J_eq(1:min(numel(J_eq), n));
        if numel(J_eq) < n
            J_eq = [J_eq, zeros(1, n - numel(J_eq))];
        end
    end
    tau_Jm = qdd_s .* repmat(J_eq, M, 1);
end
if ~isempty(opts.friction_params)
    tau_f = subtract_friction_rotor_torque(qd_s, zeros(M, n), opts.friction_params, []);
end
if ~isempty(opts.tau_bias)
    tb = opts.tau_bias(:)';
    if isscalar(tb), tb = repmat(tb, 1, n); end
    tau_bias_vec = repmat(tb, M, 1);
end
tau_comp = tau_Jm + tau_f + tau_bias_vec;
tau_id = tau_s - tau_comp;

if opts.do_plot
    plot_joints = opts.plot_joints;
    plot_joints = plot_joints(plot_joints >= 1 & plot_joints <= n);
    if isempty(plot_joints), plot_joints = 1; end
    nj = numel(plot_joints);
    figure('Name', '低通预处理前后对比', 'Position', [60 60 1200 780]);
    for ii = 1:nj
        j = plot_joints(ii);
        subplot(4, nj, ii);
        plot(t_out, q_win(:, j), 'Color', [0.7 0.7 0.7]); hold on;
        plot(t_out, q_s(:, j), 'b');
        title(sprintf('q%d', j)); grid on; hold off;

        subplot(4, nj, nj + ii);
        plot(t_out, qd_raw(idx, j), 'Color', [0.7 0.7 0.7]); hold on;
        plot(t_out, qd_s(:, j), 'b');
        title(sprintf('qd%d', j)); grid on; hold off;

        subplot(4, nj, 2 * nj + ii);
        qdd_raw_win = local_center_diff(qd_raw(idx, j), dt_full);
        plot(t_out, qdd_raw_win, 'Color', [0.7 0.7 0.7]); hold on;
        plot(t_out, qdd_s(:, j), 'b');
        title(sprintf('qdd%d', j)); grid on; hold off;

        subplot(4, nj, 3 * nj + ii);
        plot(t_out, tau_win(:, j), 'Color', [0.7 0.7 0.7]); hold on;
        plot(t_out, tau_s(:, j), 'b');
        title(sprintf('tau%d', j)); grid on; hold off;
    end
end

aux = struct();
aux.tau_s = tau_s;
aux.tau_Jm = tau_Jm;
aux.tau_f = tau_f;
aux.tau_bias = tau_bias_vec;
aux.tau_comp = tau_comp;
end

function d = local_center_diff(x, dt)
N = numel(x);
d = zeros(N, 1);
if N < 2
    return;
end
d(1) = (x(2) - x(1)) / dt;
for k = 2:N-1
    d(k) = (x(k + 1) - x(k - 1)) / (2 * dt);
end
d(N) = (x(N) - x(N - 1)) / dt;
end

