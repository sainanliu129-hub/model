function [t_out, q_s, qd_s, qdd_s, tau_id, aux] = preprocess_id_data(t, q, qd_raw, tau_raw, opts)
% preprocess_id_data  辨识前数据预处理：先 SG 求导（从 q 得 q/qd/qdd）→ 时间轴裁剪 → 滤 τ → 构造 τ_id
%
% 流程（先 SG 求导，再时间轴裁剪，与「辨识前数据预处理流程」文档一致）：
%   Step 2-3: 先 SG 求导：仅从位置 q（全段）经 Savitzky-Golay 得到 q_s, qd_s, qdd_s（三者同源）
%   Step 1: 再时间轴统一与裁剪（去首尾异常、饱和/掉包段），得到裁减后 q_s, qd_s, qdd_s
%   Step 4: 对 τ 做零相位低通（Butterworth + filtfilt）
%   Step 5: τ_id = τ_s − J_eq·qdd_s − τ_f(qd_s) − τ_bias
%
% 输入：
%   t       - N×1 时间 (s)
%   q       - N×n 关节位置 (rad)，主链路
%   qd_raw  - N×n 关节速度 (rad/s)，可选参考，不参与主链路
%   tau_raw - N×n 力矩 (N·m)，如 K_t * 电流
%   opts    - 可选，结构体：
%     时间与裁剪：
%       .t_start_s, .t_end_s     - 有效段时间范围 (s)，缺省用全段
%       .trim_start_s, .trim_end_s - 或首尾裁掉秒数
%       .trim_start_n, .trim_end_n - 或首尾裁掉点数
%       .remove_nan_inf           - true 时删掉任一带 NaN/Inf 的行（默认 true）
%       .max_abs_q, .max_abs_qd, .max_abs_tau - 超过则视为异常可删（[]=不判）
%     Savitzky-Golay（从 q 得到 q_s, qd_s, qdd_s）：
%       .sg_order   - 多项式阶次，默认 4
%       .sg_frame   - 窗长（奇数），默认 41（500Hz 约 0.08s）
%       .Fs         - 采样频率 (Hz)，缺省由 dt 估计
%    力矩滤波：
%       .tau_lowpass_fc_Hz - 截止频率 (Hz)，默认 15
%       .tau_lowpass_order - Butterworth 阶数，默认 2
%    补偿项（构造 τ_id）：
%       .J_eq       - 转子惯量 1×n 或标量 (kg·m²)，缺省 0
%       .friction_params - 摩擦参数结构体（见 subtract_friction_rotor_torque），缺省 0
%       .tau_bias   - 常值偏置 1×n 或标量，缺省 0
%    绘图（每步前后对比）：
%       .do_plot    - true 时每步后画图对比（默认 false）
%       .plot_joints - 参与绘图的关节下标，默认 [1 2 3]
%
% 输出：
%   t_out   - 裁剪并有效段后的时间 (M×1)
%   q_s     - 平滑位置 (M×n)
%   qd_s    - 由 q 经 SG 求导得到的速度 (M×n)
%   qdd_s   - 由 q 经 SG 求二阶导得到的加速度 (M×n)
%   tau_id  - 辨识用目标力矩 τ_id = τ_s − J_eq·qdd_s − τ_f − τ_bias (M×n)
%   aux     - 结构体，便于验证：.tau_s, .tau_Jm, .tau_f, .tau_bias, .tau_comp
%
% 使用：先预处理，再送入 continuous_window_id_data 时关闭内部 qd/qdd 重算，仅做时间窗。
% 验证：可画 aux.tau_Jm = J_eq·qdd_s 与原始 τ 对比，若 τ_Jm 过毛说明 qdd 尚需加强平滑。

if nargin < 5
    opts = struct();
end
t = t(:);
N = numel(t);
n = size(q, 2);
if size(q, 1) ~= N || size(tau_raw, 1) ~= N
    error('preprocess_id_data: t, q, tau_raw 行数须一致。');
end
if isempty(qd_raw), qd_raw = zeros(N, n); end
if size(qd_raw, 1) ~= N || size(qd_raw, 2) ~= n
    error('preprocess_id_data: qd_raw 须为 N×n。');
end

% ----- 默认选项 -----
if ~isfield(opts, 't_start_s'),       opts.t_start_s = []; end
if ~isfield(opts, 't_end_s'),         opts.t_end_s = []; end
if ~isfield(opts, 'trim_start_s'),    opts.trim_start_s = 0; end
if ~isfield(opts, 'trim_end_s'),      opts.trim_end_s = 0; end
if ~isfield(opts, 'trim_start_n'),    opts.trim_start_n = 0; end
if ~isfield(opts, 'trim_end_n'),      opts.trim_end_n = 0; end
if ~isfield(opts, 'remove_nan_inf'), opts.remove_nan_inf = true; end
if ~isfield(opts, 'max_abs_q'),       opts.max_abs_q = []; end
if ~isfield(opts, 'max_abs_qd'),      opts.max_abs_qd = []; end
if ~isfield(opts, 'max_abs_tau'),     opts.max_abs_tau = []; end
if ~isfield(opts, 'sg_order'),        opts.sg_order = 4; end
if ~isfield(opts, 'sg_frame'),        opts.sg_frame = 41; end
if ~isfield(opts, 'Fs'),              opts.Fs = []; end
if ~isfield(opts, 'tau_lowpass_fc_Hz'), opts.tau_lowpass_fc_Hz = 15; end
if ~isfield(opts, 'tau_lowpass_order'), opts.tau_lowpass_order = 2; end
if ~isfield(opts, 'J_eq'),            opts.J_eq = []; end
if ~isfield(opts, 'friction_params'), opts.friction_params = []; end
if ~isfield(opts, 'tau_bias'),        opts.tau_bias = []; end
if ~isfield(opts, 'do_plot'),         opts.do_plot = false; end
if ~isfield(opts, 'plot_joints'),    opts.plot_joints = [1 2 3]; end
plot_joints = opts.plot_joints;
plot_joints = plot_joints(plot_joints >= 1 & plot_joints <= n);
if isempty(plot_joints), plot_joints = 1; end

% 确保窗长为奇数
frame = max(3, opts.sg_frame);
if mod(frame, 2) == 0, frame = frame - 1; end
ord = min(opts.sg_order, frame - 1);

% 全段 dt/Fs（供 SG 用）
dt_full = median(diff(t));
if dt_full <= 0 || isnan(dt_full), dt_full = 0.002; end
Fs_full = opts.Fs;
if isempty(Fs_full), Fs_full = 1 / dt_full; end
if N < frame
    error('preprocess_id_data: 全段样本数 %d 小于 SG 窗长 %d。', N, frame);
end

%% Step 2-3: 先 SG 求导（从全段 q 得到 q_s, qd_s, qdd_s）
[~, G] = sgolay(ord, frame);
ker1 = G(end:-1:1, 2);
ker2 = G(end:-1:1, 3);
q_s_full  = zeros(N, n);
qd_s_full = zeros(N, n);
qdd_s_full = zeros(N, n);
for j = 1:n
    q_s_full(:, j)  = sgolayfilt(q(:, j), ord, frame);
    qd_s_full(:, j) = conv(q(:, j), ker1, 'same') / dt_full;
    qdd_s_full(:, j) = conv(q(:, j), ker2, 'same') / (dt_full^2);
end

%% Step 1: 再时间轴统一与裁剪（去首尾异常、饱和/掉包段）
idx = true(N, 1);
% 时间范围
if ~isempty(opts.t_start_s), idx = idx & (t >= opts.t_start_s); end
if ~isempty(opts.t_end_s),   idx = idx & (t <= opts.t_end_s); end
% 首尾按秒数裁
if opts.trim_start_s > 0
    t0 = t(find(idx, 1, 'first'));
    idx = idx & (t >= t0 + opts.trim_start_s);
end
if opts.trim_end_s > 0
    t1 = t(find(idx, 1, 'last'));
    idx = idx & (t <= t1 - opts.trim_end_s);
end
% 首尾按点数裁
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
% NaN/Inf
if opts.remove_nan_inf
    bad = any(~isfinite(q), 2) | any(~isfinite(tau_raw), 2);
    idx = idx & ~bad;
end
% 饱和/异常幅值
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

t_out   = t(idx);
q_win   = q(idx, :);
qd_raw_win = qd_raw(idx, :);
tau_win = tau_raw(idx, :);
q_s     = q_s_full(idx, :);
qd_s    = qd_s_full(idx, :);
qdd_s   = qdd_s_full(idx, :);
M = size(q_win, 1);
if M < frame
    error('preprocess_id_data: 裁剪后样本数 %d 小于 SG 窗长 %d。', M, frame);
end
dt = median(diff(t_out));
if dt <= 0 || isnan(dt), dt = 0.002; end
Fs = opts.Fs;
if isempty(Fs), Fs = 1 / dt; end

% ----- Step 1 绘图：时间轴裁剪前后对比（先 SG 求导，再裁剪） -----
if opts.do_plot
    fig1 = figure('Name', 'Step1_裁剪前后对比', 'Position', [50 50 900 500]);
    nj = numel(plot_joints);
    for ii = 1:nj
        j = plot_joints(ii);
        subplot(2, nj, ii);
        plot(t, q(:, j), 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8); hold on;
        plot(t_out, q_win(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('q (rad)'); title(sprintf('关节 %d 位置', j)); legend('原始', '裁剪后'); grid on; hold off;
        subplot(2, nj, nj + ii);
        plot(t, tau_raw(:, j), 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8); hold on;
        plot(t_out, tau_win(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('\tau (N\cdotm)'); title(sprintf('关节 %d 力矩', j)); legend('原始', '裁剪后'); grid on; hold off;
    end
    sgtitle(sprintf('Step 1 时间轴裁剪（先 SG 求导再裁剪，保留 %d/%d 点）', M, N));
end

% ----- Step 2-3 绘图：位置/速度/加速度 平滑前后对比 -----
if opts.do_plot
    qdd_raw_win = zeros(M, n);
    for j = 1:n
        for k = 2:M-1
            qdd_raw_win(k, j) = (qd_raw_win(k+1, j) - qd_raw_win(k-1, j)) / (2*dt);
        end
        qdd_raw_win(1, j) = qdd_raw_win(2, j);
        qdd_raw_win(M, j) = qdd_raw_win(M-1, j);
    end
    fig2 = figure('Name', 'SG求导前后对比', 'Position', [60 60 950 700]);
    nj = numel(plot_joints);
    for ii = 1:nj
        j = plot_joints(ii);
        subplot(3, nj, ii);
        plot(t_out, q_win(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
        plot(t_out, q_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('q (rad)'); title(sprintf('关节%d 位置', j)); legend('SG前(原始)', 'SG后(平滑)'); grid on; hold off;
        subplot(3, nj, nj + ii);
        plot(t_out, qd_raw_win(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
        plot(t_out, qd_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('q dot (rad/s)'); title(sprintf('关节%d 速度', j)); legend('SG求导前(差分)', 'SG求导后'); grid on; hold off;
        subplot(3, nj, 2*nj + ii);
        plot(t_out, qdd_raw_win(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
        plot(t_out, qdd_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('q ddot (rad/s^2)'); title(sprintf('关节%d 加速度', j)); legend('SG求导前(差分)', 'SG求导后'); grid on; hold off;
    end
    sgtitle('SG 求导前后对比（先全段 SG 求导，再 Step 1 裁剪；本图为裁剪后段）');
end

%% Step 4: 力矩零相位低通
fc = opts.tau_lowpass_fc_Hz;
ord_tau = opts.tau_lowpass_order;
tau_s = zeros(M, n);
if fc > 0 && Fs > 2*fc
    [b, a] = butter(ord_tau, fc / (Fs/2));
    for j = 1:n
        tau_s(:, j) = filtfilt(b, a, tau_win(:, j));
    end
else
    tau_s = tau_win;
end

% ----- Step 4 绘图：力矩滤波前后对比 -----
if opts.do_plot
    fig3 = figure('Name', 'Step4_力矩滤波前后对比', 'Position', [70 70 900 400]);
    nj = numel(plot_joints);
    for ii = 1:nj
        j = plot_joints(ii);
        subplot(1, nj, ii);
        plot(t_out, tau_win(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
        plot(t_out, tau_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('\tau (N\cdotm)'); title(sprintf('关节 %d', j));
        legend('滤波前', sprintf('滤波后 (%d Hz)', opts.tau_lowpass_fc_Hz)); grid on; hold off;
    end
    sgtitle(sprintf('Step 4 力矩零相位低通 (Butterworth %d 阶, %d Hz)', opts.tau_lowpass_order, opts.tau_lowpass_fc_Hz));
end

% ----- 滤波前后全关节：q, qd, qdd, tau（4 行 × 6 列，使用 Step 2-3 中已算的 qdd_raw_win） -----
if opts.do_plot
    fig_all = figure('Name', '滤波前后全关节_q_qd_qdd_tau', 'Position', [50 50 1400 900]);
    for j = 1:n
        subplot(4, n, j);
        plot(t_out, q_win(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_out, q_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad'); title(sprintf('q 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, n, n + j);
        plot(t_out, qd_raw_win(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_out, qd_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad/s'); title(sprintf('qd 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, n, 2*n + j);
        plot(t_out, qdd_raw_win(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_out, qdd_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('rad/s^2'); title(sprintf('qdd 关节%d', j)); legend('滤波前', 'SG后'); grid on; hold off;
        subplot(4, n, 3*n + j);
        plot(t_out, tau_win(:, j), 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8); hold on;
        plot(t_out, tau_s(:, j), 'b', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('\\tau 关节%d', j)); legend('滤波前', '低通后'); grid on; hold off;
    end
    sgtitle('滤波前后对比：q/qd/qdd（SG 求导）与 \\tau（Butterworth 低通）');
end

%% Step 5: τ_id = τ_s − J_eq·qdd_s − τ_f(qd_s) − τ_bias
tau_Jm = zeros(M, n);
tau_f  = zeros(M, n);
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
    % 保证与 qdd_s (M×n) 同尺寸再逐元乘，避免旧版 MATLAB 不广播
    tau_Jm = qdd_s .* repmat(J_eq, M, 1);
end
if ~isempty(opts.friction_params)
    tau_comp_f = subtract_friction_rotor_torque(qd_s, zeros(M, n), opts.friction_params, []);
    tau_f = tau_comp_f;  % 仅摩擦，不含转子
end
if ~isempty(opts.tau_bias)
    tb = opts.tau_bias(:)';
    if isscalar(tb), tb = repmat(tb, 1, n); end
    tau_bias_vec = repmat(tb, M, 1);
end
tau_comp = tau_Jm + tau_f + tau_bias_vec;
tau_id = tau_s - tau_comp;

% ----- Step 5 绘图：辨识目标力矩各分量 -----
if opts.do_plot
    fig4 = figure('Name', 'Step5_辨识目标力矩分解', 'Position', [80 80 950 650]);
    nj = numel(plot_joints);
    for ii = 1:nj
        j = plot_joints(ii);
        subplot(2, nj, ii);
        plot(t_out, tau_s(:, j), 'b', 'LineWidth', 1); hold on;
        plot(t_out, tau_Jm(:, j), 'r', 'LineWidth', 0.8);
        plot(t_out, tau_f(:, j), 'g', 'LineWidth', 0.8);
        plot(t_out, tau_id(:, j), 'k', 'LineWidth', 1.2);
        xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('关节 %d', j));
        legend('\tau_s', 'J_{eq}\ddot{q}', '\tau_f', '\tau_{id}'); grid on; hold off;
        subplot(2, nj, nj + ii);
        plot(t_out, tau_win(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
        plot(t_out, tau_id(:, j), 'k', 'LineWidth', 1);
        xlabel('t (s)'); ylabel('N\cdotm'); title(sprintf('关节 %d 原始 vs \\tau_{id}', j));
        legend('原始 \tau', '\tau_{id}'); grid on; hold off;
    end
    sgtitle('Step 5 \\tau_{id} = \\tau_s - J_{eq}\\ddot{q}_s - \\tau_f - \\tau_{bias}');
end

aux = struct();
aux.tau_s    = tau_s;
aux.tau_Jm   = tau_Jm;
aux.tau_f    = tau_f;
aux.tau_bias = tau_bias_vec;
aux.tau_comp = tau_comp;
end
