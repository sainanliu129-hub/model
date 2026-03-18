function Result = identify_inertia_segmented_sinusoidal(data_file, K_tau, tau_c, b, varargin)
% identify_inertia_segmented_sinusoidal  分段正弦轨迹惯量辨识（方案文档 8 节）
%
% 目标：τ_net = J·qdd，每段单独拟合 J_i，有效段取中位数得 J。
% 适用：分段定频正弦位置轨迹（如 10 段 × 6 s，0.1~2 Hz）。
%
% 输入:
%   data_file, K_tau, tau_c, b  同 identify_inertia_from_data
%   varargin 可选:
%     'tau_is_current', true/false
%     'SegmentDuration', 6      每段时长 (s)，默认 6
%     'NumSegments', 10          段数，默认 10（总时长≈SegmentDuration*NumSegments）
%     'MiddleFrac', 0.6          每段取中间比例（去前后各20%），默认 0.6
%     'R2Min', 0.6               有效段要求 R²>此值，默认 0.6（提高可提升精度）
%     'AccelMeanMin', 0          有效段要求 mean(|qdd|)>此值，0=不要求；建议 15~30 筛掉低 SNR 段
%     'FitWithBias', true        段内拟合 τ_net=J*qdd+c 取 J，可削弱摩擦常值残差，默认 true
%     'JMethod', 'weighted_mean' 有效段 J 汇总：'median' 或 'weighted_mean'（按 R² 加权），默认 weighted_mean
%     'VelocityFilter','lowpass', 'LowpassCutoffHz', 12  建议 10~15 Hz，过高 q̈ 噪声大
%     'DoPlot', true             是否绘制「惯量辨识」「整体拟合图」，默认 true
%     'TauCPos','TauCNeg' 或 Stribeck 参数
%     'DataFormat', 'python_upper'
%
% 输出 Result:
%   .I_a          最终惯量（有效段 J_i 的中位数）
%   .I_a_R2       有效段 R² 的中位数（或整体）
%   .I_a_RMSE     -
%   .nUsed       有效段数
%   .J_per_segment  每段拟合的 J_i
%   .R2_per_segment 每段 R²
%   .valid        有效段逻辑索引
%   .J_mean, .J_std  有效段 J_i 的均值、标准差

p = inputParser;
addParameter(p, 'tau_is_current', false, @islogical);
addParameter(p, 'SegmentDuration', 6, @isnumeric);
addParameter(p, 'NumSegments', 10, @isnumeric);
addParameter(p, 'MiddleFrac', 0.6, @isnumeric);
addParameter(p, 'R2Min', 0.6, @isnumeric);
addParameter(p, 'AccelMeanMin', 0, @isnumeric);
addParameter(p, 'FitWithBias', true, @islogical);
addParameter(p, 'JMethod', 'weighted_mean', @ischar);   % 'median' | 'weighted_mean'
addParameter(p, 'VelocityFilter', 'lowpass', @ischar);
addParameter(p, 'LowpassCutoffHz', 12, @isnumeric);
addParameter(p, 'DoPlot', true, @islogical);
addParameter(p, 'TauCPos', NaN, @isnumeric);
addParameter(p, 'TauCNeg', NaN, @isnumeric);
addParameter(p, 'TauS', NaN, @isnumeric);
addParameter(p, 'TauC', NaN, @isnumeric);
addParameter(p, 'Vs', NaN, @isnumeric);
addParameter(p, 'Tau0', NaN, @isnumeric);
addParameter(p, 'DataFormat', 'default', @ischar);
addParameter(p, 'VelocityUnit', 'rad/s', @ischar);   % 兼容 run_full 传入，本函数未使用
parse(p, varargin{:});
opts = p.Results;

use_stribeck = ~isnan(opts.TauS) && ~isnan(opts.TauC) && ~isnan(opts.Vs) && ~isnan(opts.Tau0);
use_asym = ~isnan(opts.TauCPos) && ~isnan(opts.TauCNeg) && ~use_stribeck;

if strcmp(opts.DataFormat, 'python_upper')
    cur = fileparts(mfilename('fullpath'));
    up = fullfile(cur, '..', '..', 'utility_function');
    if exist(up, 'dir') && isempty(which('read_python_upper_scan_csv'))
        addpath(up);
    end
    data = read_python_upper_scan_csv(data_file);
    opts.tau_is_current = true;
    t_data = data(:, 1);
    tau_data = data(:, 2);
    qd_act = data(:, 3);
else
    data = readmatrix(data_file);
    if size(data, 2) >= 3
        t_data = data(:, 1);
        tau_data = data(:, 2);
        qd_act = data(:, 3);
        if size(data, 2) >= 5
            qd_act = data(:, 4);
            tau_data = data(:, 5);
        end
    else
        error('数据至少需要 3 列 (t, i或τ, qd)');
    end
end

t_data = t_data(:);
tau_data = tau_data(:);
qd_act = qd_act(:);
N = length(t_data);
if N < 10
    error('数据点过少');
end

Ts = mean(diff(t_data));
Fs = 1 / Ts;
fprintf('===== 惯量辨识（分段正弦方案）=====\n');
fprintf('数据: %d 点, 采样 %.2f Hz, 时长 %.2f s\n', N, Fs, max(t_data)-min(t_data));

% 速度滤波
qd_filt = qd_act;
if strcmpi(opts.VelocityFilter, 'lowpass') && exist('lowpass', 'file')
    qd_filt = lowpass(qd_act, opts.LowpassCutoffHz, Fs);
    fprintf('  速度低通滤波: %.1f Hz\n', opts.LowpassCutoffHz);
end

% 加速度（实际时间差）
dt = diff(t_data);
dt(end+1) = dt(end);
qdd = diff(qd_filt) ./ dt(1:end-1);
qdd = [qdd(1); qdd];
if length(qdd) < N
    qdd(N) = qdd(end);
end

% τ 与 τ_net
if opts.tau_is_current
    tau_real = K_tau * tau_data;
else
    tau_real = tau_data;
end
if use_stribeck
    tau_f = (opts.TauC + (opts.TauS - opts.TauC)*exp(-(abs(qd_filt)/opts.Vs).^2)).*sign(qd_filt) + b*qd_filt + opts.Tau0;
elseif use_asym
    tau_f = (qd_filt > 0).*(opts.TauCPos + b*qd_filt) + (qd_filt < 0).*(opts.TauCNeg + b*qd_filt);
else
    tau_f = tau_c * sign(qd_filt) + b * qd_filt;
end
tau_net = tau_real - tau_f;

% 相对时间（从 0 开始）
t_rel = t_data - min(t_data);
T_total = max(t_rel);
seg_dur = opts.SegmentDuration;
num_seg = opts.NumSegments;
if T_total < num_seg * seg_dur * 0.5
    num_seg = max(1, floor(T_total / seg_dur));
    fprintf('  总时长 < %.0f s，按 %.0f 段处理\n', num_seg*seg_dur, num_seg);
end

J_i = nan(num_seg, 1);
R2_i = nan(num_seg, 1);
RMSE_i = nan(num_seg, 1);
mean_abs_qdd_i = nan(num_seg, 1);

for i = 1:num_seg
    t_lo = (i-1) * seg_dur;
    t_hi = i * seg_dur;
    idx_seg = find(t_rel >= t_lo & t_rel < t_hi);
    if isempty(idx_seg), continue; end
    n = length(idx_seg);
    n_keep = max(1, round(n * opts.MiddleFrac));
    n_skip = floor((n - n_keep) / 2);
    idx_mid = idx_seg(n_skip + (1:n_keep));
    qdd_s = qdd(idx_mid);
    tau_s = tau_net(idx_mid);
    mean_abs_qdd_i(i) = mean(abs(qdd_s));
    sum_qdd2 = sum(qdd_s.^2);
    if sum_qdd2 < 1e-12, continue; end
    if opts.FitWithBias
        % τ_net ≈ J*qdd + c（最小二乘），c 为段内常值残差（含摩擦模型误差等），只取 J 作为惯量
        X = [qdd_s, ones(size(qdd_s))];
        beta = X \ tau_s;   % 线性最小二乘
        J_ii = beta(1);
        tau_pred = X * beta;
    else
        % 过原点最小二乘：min sum(τ_net - J*qdd)^2 => J = sum(qdd.*τ_net)/sum(qdd^2)
        J_ii = sum(qdd_s .* tau_s) / sum_qdd2;
        tau_pred = J_ii * qdd_s;
    end
    ss_res = sum((tau_s - tau_pred).^2);
    ss_tot = sum((tau_s - mean(tau_s)).^2);
    R2_ii = 1 - ss_res / max(ss_tot, 1e-12);
    RMSE_ii = sqrt(mean((tau_s - tau_pred).^2));
    J_i(i) = J_ii;
    R2_i(i) = R2_ii;
    RMSE_i(i) = RMSE_ii;
end

valid = (R2_i > opts.R2Min) & ~isnan(J_i);
if opts.AccelMeanMin > 0
    valid = valid & (mean_abs_qdd_i >= opts.AccelMeanMin);
end
J_valid = J_i(valid);
R2_valid = R2_i(valid);

if all(isnan(J_i)) || isempty(J_valid)
    Result.I_a = NaN;
    Result.I_a_R2 = NaN;
    Result.I_a_RMSE = NaN;
    Result.nUsed = 0;
    Result.J_per_segment = J_i;
    Result.R2_per_segment = R2_i;
    Result.valid = valid;
    Result.J_mean = NaN;
    Result.J_std = NaN;
    fprintf('  无有效段（R²>%.2f 且满足加速度条件）。\n', opts.R2Min);
    % 诊断：各段 R² 与 mean(|q̈|)，便于放宽 R2Min/AccelMeanMin
    ok_r2 = ~isnan(J_i);
    if any(ok_r2)
        fprintf('  诊断: 各段 R² = '); fprintf('%.3f ', R2_i(ok_r2)); fprintf('\n');
        fprintf('  诊断: 各段 mean(|q̈|) = '); fprintf('%.1f ', mean_abs_qdd_i(ok_r2)); fprintf(' (rad/s²)\n');
        fprintf('  若仍无有效段可试: R2Min=0.4, AccelMeanMin=0\n');
    else
        fprintf('  诊断: 所有段 J 均为 NaN（段内 q̈ 过小或数据异常）\n');
    end
    return;
end

if strcmpi(opts.JMethod, 'weighted_mean') && numel(J_valid) > 0
    w = max(R2_valid, 0.01);
    J_final = sum(J_valid .* w) / sum(w);
else
    J_final = median(J_valid);
end
Result.I_a = J_final;
Result.I_a_R2 = median(R2_i(valid));
Result.I_a_RMSE = mean(RMSE_i(valid));
Result.nUsed = sum(valid);
Result.J_per_segment = J_i;
Result.R2_per_segment = R2_i;
Result.valid = valid;
Result.J_mean = mean(J_valid);
Result.J_std = std(J_valid);

fprintf('  有效段 %d / %d（R²>%.2f）\n', sum(valid), numel(J_i), opts.R2Min);
if strcmpi(opts.JMethod, 'weighted_mean')
    fprintf('  J = weighted_mean(J_i, by R²) = %.6f kg·m²\n', J_final);
else
    fprintf('  J = median(J_i) = %.6f kg·m²\n', J_final);
end
fprintf('  J_mean = %.6f, J_std = %.6f\n', Result.J_mean, Result.J_std);
fprintf('  有效段 R² 中位数 = %.4f\n', Result.I_a_R2);

% 与 identify_inertia_from_data 一致：绘制「惯量辨识」「整体拟合图」供保存
if opts.DoPlot && ~isnan(J_final)
    % 惯量辨识图：有效段内 (qdd, τ_net) 散点 + 拟合线 I_a*qdd
    qdd_plot = [];
    tau_net_plot = [];
    for i = 1:num_seg
        if ~valid(i), continue; end
        t_lo = (i-1) * seg_dur;
        t_hi = i * seg_dur;
        idx_seg = find(t_rel >= t_lo & t_rel < t_hi);
        if isempty(idx_seg), continue; end
        n = length(idx_seg);
        n_keep = max(1, round(n * opts.MiddleFrac));
        n_skip = floor((n - n_keep) / 2);
        idx_mid = idx_seg(n_skip + (1:n_keep));
        qdd_plot = [qdd_plot; qdd(idx_mid)];
        tau_net_plot = [tau_net_plot; tau_net(idx_mid)];
    end
    if ~isempty(qdd_plot)
        figure('Name', '惯量辨识');
        plot(qdd_plot, tau_net_plot, 'b.', 'MarkerSize', 4);
        hold on;
        qdd_line = linspace(min(qdd_plot), max(qdd_plot), 200);
        plot(qdd_line, J_final * qdd_line, 'r-', 'LineWidth', 2);
        grid on; xlabel('角加速度 \\ddot{q} (rad/s^2)'); ylabel('净力矩 \\tau_{net} (N\\cdot m)');
        title(sprintf('惯量辨识（分段正弦）: I_a = %.6f kg\\cdot m^2, R^2 中位数 = %.4f', J_final, Result.I_a_R2));
        legend('有效段数据', 'I_a \\times \\ddot{q}', 'Location', 'best');
        % 与 run_full 保存列表一致：加速度拟合图（内容同惯量辨识）
        figure('Name', '加速度拟合图');
        plot(qdd_plot, tau_net_plot, 'b.', 'MarkerSize', 4);
        hold on;
        plot(qdd_line, J_final * qdd_line, 'r-', 'LineWidth', 2);
        grid on; xlabel('角加速度 \\ddot{q} (rad/s^2)'); ylabel('净力矩 \\tau_{net} (N\\cdot m)');
        title(sprintf('加速度拟合（分段正弦）: I_a = %.6f kg\\cdot m^2', J_final));
        legend('有效段数据', 'I_a \\times \\ddot{q}', 'Location', 'best');
    end
    % 整体拟合图：实际力矩 vs 模型力矩 I_a*qdd+τ_f
    tau_model = J_final * qdd + tau_f;
    figure('Name', '整体拟合图');
    subplot(2,1,1);
    plot(t_data, tau_real, 'b-', 'LineWidth', 0.8); hold on;
    plot(t_data, tau_model, 'r-', 'LineWidth', 0.8);
    grid on; xlabel('时间 (s)'); ylabel('力矩 (N\\cdot m)');
    title('整体拟合：实际力矩 vs 模型力矩（分段正弦 I_a\\ddot{q}+\\tau_f）');
    legend('实际力矩 \\tau_{real}', '模型力矩 I_a\\ddot{q}+\\tau_f', 'Location', 'best');
    subplot(2,1,2);
    plot(tau_real, tau_model, 'b.', 'MarkerSize', 4);
    hold on;
    ax = axis; lim = [min(ax(1),ax(3)), max(ax(2),ax(4))];
    plot(lim, lim, 'r--', 'LineWidth', 1);
    grid on; xlabel('实际力矩 \\tau_{real} (N\\cdot m)'); ylabel('模型力矩 (N\\cdot m)');
    title('整体拟合散点（斜线为理想一致）');
    legend('实测', '45° 理想线', 'Location', 'best');
end

fprintf('\n===== 分段正弦惯量辨识完成 =====\n');
end
