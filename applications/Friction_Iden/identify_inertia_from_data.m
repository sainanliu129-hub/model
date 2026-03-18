function Result = identify_inertia_from_data(data_file, K_tau, tau_c, b, varargin)
% identify_inertia_from_data  从实测数据辨识电机转子惯量 I_a
%
% 功能：读取包含加速度轨迹的实测数据，辨识电机转子惯量 I_a
%
% 输入:
%   data_file  - 数据文件路径（CSV格式）
%   K_tau      - 力矩系数 N·m/A（如果 joint_torque 是电流，需要转换）
%   tau_c      - 已辨识的库伦摩擦 N·m
%   b          - 已辨识的粘滞摩擦系数 N·m·s/rad
%   varargin   - 可选参数（名值对）:
%     'tau_is_current'  - true/false，joint_torque 列是否为电流（true）或力矩（false），默认 false
%     'Ts'              - 采样周期 s，如果数据中时间不连续则需指定，默认从数据计算
%     'VelocityFilter'  - 求 q̈ 前对 q̇ 的滤波：'none'（默认）、'lowpass'、'sgolay'（改善 q̈ 质量）
%     'LowpassCutoffHz' - VelocityFilter='lowpass' 时截止频率 Hz，默认 10
%     'SgolayOrder'     - VelocityFilter='sgolay' 时阶数，默认 3
%     'SgolayFrame'     - VelocityFilter='sgolay' 时帧长（奇数），默认 21
%     'TauShiftSamples' - 力矩相对 q̈ 的采样平移（正=τ 滞后），默认 0；若 R² 很低可试 1 或 2 做时间对齐
%     'accel_threshold' - 拟合时仅用 |q̈|>此值的点，默认 0.01 rad/s²；可设 50 等只用大加速度段
%     'AccelExcludeAbove'- 剔除 |q̈|>此值的点，不参与拟合与图，默认 Inf
%     'TauExcludeAbove'  - 剔除 |τ|>此值的点（N·m），不参与拟合与图，默认 Inf
%     'DoPlot'          - 是否绘制图形，默认 true
%     'DoPlotDiagnostic'- 是否绘制 τ_real、τ_friction、τ_net、q̈ 对比（判断净力矩是否由惯量主导），默认 false
%     'VelocityUnit'    - 3 列格式时速度单位：'rpm'（默认）或 'rad/s'
%     'TauCPos','TauCNeg' - 可选，不对称库伦（正向/负向 N·m）；同时给出则忽略 tau_c，用二者做摩擦补偿
%     'DataFormat'      - 'default' 或 'python_upper'（上位机 6/7 列，同摩擦辨识）
%     'PulseMinLen'       - 匀加/匀减速段最少样本数，不足则丢弃，默认 15
%     'AccelMeanMin'      - 只保留 |mean(qdd)| > 此值的段（rad/s²），如 300~500，默认 300
%     'AccelStdRatioMax'  - 段内 std(qdd)/|mean(qdd)| < 此比例才保留（匀加/匀减速要稳定），默认 0.3
%     'SignCheck'         - 拟合前是否做符号检查（q̈>0 时 τ_real、τ_net 应大致为正），默认 true
%     'SignCheckAccelThresh'- 符号检查用窗口：q̈ 需 > 此值 rad/s²，默认 10
%
% 拟合方式：轨迹视为 匀速 / 匀加速 / 匀减速 段。匀速段用于摩擦辨识（本函数不用）；匀加速、匀减速段
% 整段求平均得 (mean(qdd), mean(tau_net))，每段 1 点，只保留 |mean(qdd)| 足够大且段内 qdd 稳定的段，再做 tau_net = I_a*qdd 过原点回归。
%
% 数据文件格式（CSV）:
%   3 列：time, 电流或力矩, velocity(rpm 或 rad/s)
%   6/7 列（DataFormat='python_upper'）：取时间、phase_current_A（实际电流）、速度；不用 command_current_A
%   5+ 列默认：time, cmd_pos, joint_pos, joint_vel, joint_torque, ...
%
% 输出:
%   Result - 结构体:
%     .I_a      - 电机转子惯量 kg·m²
%     .I_a_R2   - 拟合 R²
%     .I_a_RMSE - 拟合 RMSE
%     .nUsed    - 用于拟合的样本数
%
% R² 很低（如 <0.3）时建议：① VelocityFilter='lowpass' 或 'sgolay' 改善 q̈；
%  ② 试 TauShiftSamples=1 或 2 做时间对齐；③ 提高 accel_threshold（如 50）只用大加速度段；
%  ④ DoPlotDiagnostic=true 看 τ_net 是否与 q̈ 形状一致。
%
% 如何判断是否滤波：
%  - 先不滤波跑一次，看拟合 R²。若 R² < 0.5，说明 τ_net 与 q̈ 线性关系弱，常见原因之一是 q̈ 由
%    速度差分得到、噪声被放大；此时建议用 VelocityFilter='lowpass' 或 'sgolay' 再跑，对比 R²。
%  - 若滤波后 R² 明显提高（如 0.2→0.6），说明滤波有效，可采用滤波结果。
%  - 若滤波后 R² 仍很低，则问题更可能是时间不同步或 τ≠I·q̈（见 DoPlotDiagnostic）。
%
% 三步骤误差来源检查（R² 很低时）：
%  ① 基线；② TauShiftSamples=1（τ 平移一帧）：若 R² 明显升高多为力矩延迟；
%  ③ accel_threshold=80（只用大加速度）：若 R² 明显升高多为小加速度段噪声/摩擦残差。
%  DoPlotDiagnostic=true 时最后一子图为 τ_net 与 I_a×q̈ 时间序列对比，可目视是否仅错位。
%
% 示例:
%   % 先辨识摩擦参数
%   Result_friction = identify_friction_from_trajectory('velocity_data.csv', 'K_tau', 0.5);
%   
%   % 再辨识惯量
%   Result_inertia = identify_inertia_from_data('acceleration_data.csv', 0.5, ...
%       Result_friction.tau_c, Result_friction.b);

%% 参数解析
p = inputParser;
addParameter(p, 'tau_is_current', false, @islogical);
addParameter(p, 'Ts', [], @isnumeric);
addParameter(p, 'VelocityFilter', 'none', @(x) ischar(x) && ismember(x, {'none', 'lowpass', 'sgolay'}));
addParameter(p, 'LowpassCutoffHz', 10, @isnumeric);
addParameter(p, 'SgolayOrder', 3, @isnumeric);
addParameter(p, 'SgolayFrame', 21, @isnumeric);  % 须为奇数
addParameter(p, 'TauShiftSamples', 0, @isnumeric);  % 力矩相对 q̈ 的采样平移
addParameter(p, 'accel_threshold', 0.01, @isnumeric);  % 拟合时仅用 |q̈|>此值的点；脉冲判定同此阈值
addParameter(p, 'AccelExcludeAbove', Inf, @isnumeric);  % 剔除 |q̈|>此值的点，不参与拟合与图；Inf=不剔除
addParameter(p, 'TauExcludeAbove', Inf, @isnumeric);    % 剔除 |τ|>此值的点（N·m），不参与拟合与图；Inf=不剔除
addParameter(p, 'PulseMinLen', 15, @isnumeric);         % 匀加/匀减速段最少样本数，不足则丢弃
addParameter(p, 'AccelMeanMin', 0, @isnumeric);        % 只保留 |mean(qdd)|>此值的段；0=仅用 accel_threshold 判段，不再额外抬门槛
addParameter(p, 'AccelStdRatioMax', Inf, @isnumeric);  % 段内 std(qdd)/|mean(qdd)|<此比例才保留；Inf=不要求稳定
addParameter(p, 'SignCheck', true, @islogical);         % 拟合前是否做符号检查（q̈>0 时 τ_real、τ_net 应大致为正）
addParameter(p, 'SignCheckAccelThresh', 10, @isnumeric); % 符号检查用的小窗口：q̈ 需 > 此值 rad/s²
addParameter(p, 'DoPlot', true, @islogical);
addParameter(p, 'DoPlotDiagnostic', false, @islogical);  % τ_real, τ_friction, τ_net, q̈ 对比
addParameter(p, 'VelocityUnit', 'rpm', @ischar);  % 3 列时速度单位：'rpm' 或 'rad/s'
addParameter(p, 'TauCPos', NaN, @isnumeric);
addParameter(p, 'TauCNeg', NaN, @isnumeric);
addParameter(p, 'TauS', NaN, @isnumeric);   % Stribeck 静摩擦峰值（与 TauC, Vs, Tau0 同用）
addParameter(p, 'TauC', NaN, @isnumeric);    % Stribeck 库伦摩擦
addParameter(p, 'Vs', NaN, @isnumeric);      % Stribeck 过渡速度 rad/s
addParameter(p, 'Tau0', NaN, @isnumeric);   % Stribeck 常值偏置
addParameter(p, 'DataFormat', 'default', @(x) ischar(x) && ismember(x, {'default', 'python_upper'}));
parse(p, varargin{:});
opts = p.Results;

use_stribeck = ~isnan(opts.TauS) && ~isnan(opts.TauC) && ~isnan(opts.Vs) && ~isnan(opts.Tau0);
use_asym = ~isnan(opts.TauCPos) && ~isnan(opts.TauCNeg) && ~use_stribeck;

% 确保 utility_function 在路径（DataFormat='python_upper' 时需 read_python_upper_scan_csv）
if strcmp(opts.DataFormat, 'python_upper')
    cur = fileparts(mfilename('fullpath'));
    up = fullfile(cur, '..', '..', 'utility_function');
    if exist(up, 'dir') && isempty(which('read_python_upper_scan_csv'))
        addpath(up);
    end
end

fprintf('===== 辨识电机转子惯量 I_a =====\n');
fprintf('数据文件: %s\n', data_file);
fprintf('力矩系数 K_tau: %.4f N·m/A\n', K_tau);
if use_stribeck
    fprintf('摩擦模型: Stribeck (τ_s=%.4f, τ_c=%.4f, v_s=%.4f, τ_0=%.4f N·m)\n', opts.TauS, opts.TauC, opts.Vs, opts.Tau0);
elseif use_asym
    fprintf('库伦摩擦（不对称）τ_c_pos: %.4f, τ_c_neg: %.4f N·m\n', opts.TauCPos, opts.TauCNeg);
else
    fprintf('库伦摩擦 τ_c: %.4f N·m\n', tau_c);
end
fprintf('粘滞摩擦系数 b: %.4f N·m·s/rad\n', b);

%% 1. 读取数据文件
fprintf('\n步骤1：读取数据文件...\n');
if ~exist(data_file, 'file')
    error('数据文件不存在: %s', data_file);
end

if strcmp(opts.DataFormat, 'python_upper')
    % 上位机 6/7 列格式（read_python_upper_scan_csv 已输出 rad/s，此处不再转换）
    data = read_python_upper_scan_csv(data_file);
    opts.tau_is_current = true;
    opts.VelocityUnit = 'rad/s';  % 读取函数内已做 rpm→rad/s
    fprintf('已按 DataFormat=''python_upper'' 读取：时间, phase_current_A, speed_rad/s → 3 列\n');
else
    % 读取CSV文件（支持无表头或带表头；支持 3 列或 5+ 列）
    try
        data = readmatrix(data_file);
    catch
        fid = fopen(data_file, 'r');
        fgetl(fid);
        fgetl(fid);
        data = [];
        while true
            line = fgetl(fid);
            if ~ischar(line), break; end
            parts = strsplit(line, ',');
            if length(parts) >= 3
                row = str2double(parts(1:min(5, length(parts))));
                if ~any(isnan(row)), data = [data; row]; end
            end
        end
        fclose(fid);
    end
    while size(data, 1) > 0 && any(isnan(data(1, :)))
        data = data(2:end, :);
    end
end

fprintf('数据大小: %d 行 × %d 列\n', size(data, 1), size(data, 2));

% 提取数据列（支持 3 列格式：time, torque, velocity）
if size(data, 2) == 3
    % 3 列格式：时间, 力矩, 电机转速（单位见 VelocityUnit）
    fprintf('检测到 3 列格式：time, torque, velocity（单位: %s）\n', opts.VelocityUnit);
    t_data = data(:, 1);
    tau_data = data(:, 2);
    qd_act = data(:, 3);
    if strcmpi(opts.VelocityUnit, 'rpm')
        qd_act = qd_act * (2*pi/60);  % rpm → rad/s
    end
    % 使速度与力矩正负一致：若多数时刻相反则对速度取反
    valid = abs(qd_act) > 0.01;
    if any(valid) && mean(sign(qd_act(valid)) .* sign(tau_data(valid))) < 0
        qd_act = -qd_act;
        fprintf('3列数据：检测到力矩与速度正负相反，已对速度取反以使二者一致。\n');
    end
    q_cmd = zeros(size(t_data));
    q_act = zeros(size(t_data));
elseif size(data, 2) >= 5
    t_data = data(:, 1);           % 时间
    q_cmd = data(:, 2);            % 指令位置
    q_act = data(:, 3);            % 实际位置
    qd_act = data(:, 4);           % 实际速度 joint_vel
    tau_data = data(:, 5);         % 力矩（或电流）joint_torque
    
    % 可选：提取扫频参数（如果存在）
    if size(data, 2) >= 9
        freq_start = data(1, 6);      % 起始频率
        freq_end = data(1, 7);         % 结束频率
        duration = data(1, 8);         % 持续时间
        amplitude_ratio = data(1, 9);  % 振幅比例
        fprintf('扫频参数: %.2f - %.2f Hz, 时长 %.2f s, 振幅比例 %.2f\n', ...
            freq_start, freq_end, duration, amplitude_ratio);
    end
else
    error('数据文件列数不足，至少需要 3 列(time,torque,velocity) 或 5 列');
end

% 计算采样周期
if isempty(opts.Ts)
    if length(t_data) > 1
        opts.Ts = t_data(2) - t_data(1);
    else
        error('无法确定采样周期，请指定 Ts 参数');
    end
end

fprintf('数据时长: %.2f s\n', t_data(end));
fprintf('采样周期: %.4f s (%.1f Hz)\n', opts.Ts, 1/opts.Ts);

%% 2. 计算加速度 q̈：可选对 q̇ 滤波后再微分（改善 q̈ 质量，缓解 R² 过低）
fprintf('\n步骤2：计算加速度 q̈...\n');
Fs = 1 / opts.Ts;
qd_for_diff = qd_act;
if strcmpi(opts.VelocityFilter, 'lowpass')
    if exist('lowpass', 'file')
        qd_for_diff = lowpass(qd_act, opts.LowpassCutoffHz, Fs);
        fprintf('  已对 q̇ 低通滤波：截止 %.1f Hz，采样 %.1f Hz\n', opts.LowpassCutoffHz, Fs);
    else
        warning('未找到 lowpass（需 Signal Processing Toolbox），跳过速度滤波');
    end
elseif strcmpi(opts.VelocityFilter, 'sgolay')
    ord = opts.SgolayOrder;
    fram = max(3, min(opts.SgolayFrame, floor(length(qd_act)/2)));  % 奇数
    if mod(fram, 2) == 0, fram = fram - 1; end
    qd_for_diff = sgolayfilt(qd_act, ord, fram);
    fprintf('  已对 q̇ Savitzky-Golay 滤波：阶数 %d，帧长 %d\n', ord, fram);
end

% 数值微分
if length(t_data) > 1
    dt_samples = diff(t_data);
    dt_mean = mean(dt_samples);
    dt_std = std(dt_samples);
    if dt_std / dt_mean < 0.01
        Ts_actual = dt_mean;
        fprintf('采样周期恒定: %.6f s\n', Ts_actual);
        qdd_numerical = diff(qd_for_diff) / Ts_actual;
        qdd_numerical = [qdd_numerical(1); qdd_numerical];
    else
        fprintf('采样周期不恒定，使用实际时间差\n');
        qdd_numerical = diff(qd_for_diff) ./ dt_samples;
        qdd_numerical = [qdd_numerical(1); qdd_numerical];
    end
else
    error('数据点不足，无法计算加速度');
end
qdd_filtered = qdd_numerical;

fprintf('速度范围: %.4f ~ %.4f rad/s\n', min(qd_act), max(qd_act));
fprintf('加速度范围: %.4f ~ %.4f rad/s²\n', min(qdd_filtered), max(qdd_filtered));

%% 3. 计算 τ_real 和 τ_friction
fprintf('\n步骤3：计算 τ_real 和 τ_friction...\n');

% τ_real：如果 joint_torque 是电流，需要转换为力矩
if opts.tau_is_current
    tau_real = K_tau * tau_data;  % τ_real = K_τ × i
else
    tau_real = tau_data;  % 已经是力矩
end

% τ_friction：用已辨识的摩擦参数（支持 Stribeck、不对称库伦、对称库伦）
if use_stribeck
    tau_friction = (opts.TauC + (opts.TauS - opts.TauC)*exp(-(abs(qd_act)/opts.Vs).^2)).*sign(qd_act) + b*qd_act + opts.Tau0;
elseif use_asym
    tau_friction = (qd_act > 0) .* (opts.TauCPos + b * qd_act) + (qd_act < 0) .* (opts.TauCNeg + b * qd_act) + (qd_act == 0) .* 0;
else
    tau_friction = tau_c * sign(qd_act) + b * qd_act;
end

tau_net = tau_real - tau_friction;  % 净力矩

% 可选：力矩相对 q̈ 的时间对齐（若 R² 很低可试 TauShiftSamples=1 或 2）
shift = round(opts.TauShiftSamples);
if shift > 0
    n = length(qdd_filtered);
    if n > shift
        tau_net = tau_net((1+shift):end);
        qdd_filtered = qdd_filtered(1:(end-shift));
        t_data = t_data(1:(end-shift));
        tau_real = tau_real((1+shift):end);
        tau_friction = tau_friction((1+shift):end);
        qd_act = qd_act(1:(end-shift));
        fprintf('  已做时间对齐：TauShiftSamples = %d（τ 相对 q̈ 滞后 %d 个采样）\n', shift, shift);
    end
end

%% 4. 符号检查 + 按脉冲取中间窗口平均后拟合 I_a（不做全段逐点回归）
fprintf('\n步骤4：符号检查与脉冲窗口平均拟合 I_a...\n');

% 剔除 |q̈|、|τ| 过大的点（不参与脉冲与拟合）
idx_exclude = false(size(qdd_filtered));
if isfinite(opts.AccelExcludeAbove)
    idx_exclude = idx_exclude | (abs(qdd_filtered) > opts.AccelExcludeAbove);
end
if isfinite(opts.TauExcludeAbove)
    idx_exclude = idx_exclude | (abs(tau_real) > opts.TauExcludeAbove);
end
thresh = max(opts.accel_threshold, 0.01);

% ----- 符号 sanity check：任选一段 q̈>0 的小窗口，检查 τ_real、τ_net 是否大致为正 -----
sign_ok = true;
if opts.SignCheck && opts.SignCheckAccelThresh > 0
    qdd_pos = qdd_filtered > opts.SignCheckAccelThresh & ~idx_exclude;
    % 找第一段连续 q̈>0 且长度>=5
    run_start = find(diff([0; qdd_pos; 0]) == 1);
    run_end   = find(diff([0; qdd_pos; 0]) == -1);
    if isempty(run_start), run_start = 1; end
    if length(run_end) < length(run_start), run_end(end+1) = length(qdd_pos)+1; end
    found = false;
    for r = 1:min(length(run_start), length(run_end))
        len = run_end(r) - run_start(r);
        if len >= 5
            idx_win = run_start(r) : (run_end(r)-1);
            m_tau_real = mean(tau_real(idx_win));
            m_tau_net  = mean(tau_net(idx_win));
            if m_tau_real <= 0
                fprintf('  [符号检查未通过] 在一段 q̈>%.1f 的窗口内 mean(τ_real)=%.4f ≤ 0，请检查数据/符号。\n', opts.SignCheckAccelThresh, m_tau_real);
                sign_ok = false;
            elseif m_tau_net <= 0
                fprintf('  [符号检查未通过] 在一段 q̈>%.1f 的窗口内扣掉摩擦后 mean(τ_net)=%.4f ≤ 0，符号或摩擦补偿可能有问题，不继续拟合。\n', opts.SignCheckAccelThresh, m_tau_net);
                sign_ok = false;
            else
                fprintf('  符号检查通过：q̈>0 段内 mean(τ_real)=%.4f, mean(τ_net)=%.4f\n', m_tau_real, m_tau_net);
            end
            found = true;
            break;
        end
    end
    if ~found
        fprintf('  未找到长度>=5 的 q̈>%.1f 段，跳过符号检查。\n', opts.SignCheckAccelThresh);
    end
end

% ----- 轨迹分段：匀速 / 匀加速 / 匀减速。匀速段用于摩擦辨识（本处不用）；匀加、匀减速段整段求平均得 (qdd_bar, tau_net_bar) 用于 I_a 拟合 -----
% 匀加速段：qdd > thresh 的连续样本；匀减速段：qdd < -thresh 的连续样本；|qdd|<=thresh 视为匀速段，不参与惯量拟合
% 每个匀加/匀减速段整段算一次平均 -> 一个 (mean(qdd), mean(tau_net)) 点；只保留 |mean(qdd)| 足够大且段内 qdd 稳定（std/|mean|<比例）的段
qdd_bar_list = [];
tau_net_bar_list = [];

% 匀加速段：qdd > thresh 的连续段，整段求平均得 1 点
pos_mask = (qdd_filtered > thresh) & ~idx_exclude;
run_start = find(diff([0; pos_mask; 0]) == 1);
run_end   = find(diff([0; pos_mask; 0]) == -1);
if length(run_end) < length(run_start), run_end(end+1) = length(pos_mask)+1; end
for r = 1:min(length(run_start), length(run_end))
    len = run_end(r) - run_start(r);
    if len < opts.PulseMinLen, continue; end
    idx_seg = run_start(r) : (run_end(r)-1);
    qdd_seg = qdd_filtered(idx_seg);
    tau_seg = tau_net(idx_seg);
    qdd_bar = mean(qdd_seg);
    tau_net_bar = mean(tau_seg);
    qdd_std_seg = std(qdd_seg);
    if numel(qdd_seg) < 2, qdd_std_seg = 0; end
    if opts.AccelMeanMin > 0 && abs(qdd_bar) < opts.AccelMeanMin, continue; end
    if isfinite(opts.AccelStdRatioMax) && qdd_std_seg >= opts.AccelStdRatioMax * max(abs(qdd_bar), 1e-9), continue; end
    qdd_bar_list(end+1) = qdd_bar;
    tau_net_bar_list(end+1) = tau_net_bar;
end
% 匀减速段：qdd < -thresh 的连续段，整段求平均得 1 点
neg_mask = (qdd_filtered < -thresh) & ~idx_exclude;
run_start = find(diff([0; neg_mask; 0]) == 1);
run_end   = find(diff([0; neg_mask; 0]) == -1);
if length(run_end) < length(run_start), run_end(end+1) = length(neg_mask)+1; end
for r = 1:min(length(run_start), length(run_end))
    len = run_end(r) - run_start(r);
    if len < opts.PulseMinLen, continue; end
    idx_seg = run_start(r) : (run_end(r)-1);
    qdd_seg = qdd_filtered(idx_seg);
    tau_seg = tau_net(idx_seg);
    qdd_bar = mean(qdd_seg);
    tau_net_bar = mean(tau_seg);
    qdd_std_seg = std(qdd_seg);
    if numel(qdd_seg) < 2, qdd_std_seg = 0; end
    if opts.AccelMeanMin > 0 && abs(qdd_bar) < opts.AccelMeanMin, continue; end
    if isfinite(opts.AccelStdRatioMax) && qdd_std_seg >= opts.AccelStdRatioMax * max(abs(qdd_bar), 1e-9), continue; end
    qdd_bar_list(end+1) = qdd_bar;
    tau_net_bar_list(end+1) = tau_net_bar;
end

qdd_bar_list = qdd_bar_list(:);
tau_net_bar_list = tau_net_bar_list(:);
n_seg = length(qdd_bar_list);
if opts.AccelMeanMin > 0 || isfinite(opts.AccelStdRatioMax)
    fprintf('  匀加/匀减速段整段平均：保留 |mean(qdd)|>%.0f 且 std/|mean|<%.2f -> 共 %d 段（每段 1 点）\n', ...
        opts.AccelMeanMin, opts.AccelStdRatioMax, n_seg);
else
    fprintf('  匀加/匀减速段整段平均（|qdd|>accel_threshold 即保留）-> 共 %d 段（每段 1 点）\n', n_seg);
end

if sign_ok && n_seg >= 3
    % 线性回归过原点：tau_net_bar = I_a * qdd_bar
    I_a_estimated = (qdd_bar_list' * tau_net_bar_list) / (qdd_bar_list' * qdd_bar_list + 1e-12);
    y_fit = tau_net_bar_list;
    y_fit_pred = I_a_estimated * qdd_bar_list;
    ss_res = sum((y_fit - y_fit_pred).^2);
    ss_tot = sum((y_fit - mean(y_fit)).^2);
    R2_Ia = 1 - ss_res / max(ss_tot, 1e-12);
    RMSE_Ia = sqrt(mean((y_fit - y_fit_pred).^2));
    fprintf('  电机转子惯量 I_a = %.6f kg·m²\n', I_a_estimated);
    fprintf('  拟合 R² = %.4f（基于 %d 个匀加/匀减速段）\n', R2_Ia, n_seg);
    fprintf('  拟合 RMSE = %.6f N·m\n', RMSE_Ia);
    Result.I_a = I_a_estimated;
    Result.I_a_R2 = R2_Ia;
    Result.I_a_RMSE = RMSE_Ia;
    Result.nUsed = n_seg;
    x_fit = qdd_bar_list;
    y_fit = tau_net_bar_list;
    idx_valid = false(size(qdd_filtered));
else
    if ~sign_ok
        fprintf('  因符号检查未通过，跳过惯量拟合。\n');
    else
        fprintf('  匀加/匀减速段有效点不足（需至少 3 段），无法拟合。\n');
    end
    I_a_estimated = NaN;
    R2_Ia = NaN;
    RMSE_Ia = NaN;
    n_seg = 0;
    Result.I_a = NaN;
    Result.I_a_R2 = NaN;
    Result.I_a_RMSE = NaN;
    Result.nUsed = 0;
    x_fit = [];
    y_fit = [];
    idx_valid = false(size(qdd_filtered));
end

if Result.nUsed >= 3
    
    %% 5. 可视化：截取后数据、加速度拟合图、惯量辨识图、整体拟合图
    if opts.DoPlot
        idx_exclude_display = false(size(tau_real));
        if isfinite(opts.AccelExcludeAbove)
            idx_exclude_display = idx_exclude_display | (abs(qdd_filtered) > opts.AccelExcludeAbove);
        end
        if isfinite(opts.TauExcludeAbove)
            idx_exclude_display = idx_exclude_display | (abs(tau_real) > opts.TauExcludeAbove);
        end
        qdd_display = qdd_filtered;
        qdd_display(idx_exclude_display) = NaN;
        tau_real_display = tau_real;
        tau_real_display(idx_exclude_display) = NaN;

        % 截取后数据：加速度 + 电流（或力矩）
        figure('Name', '截取后数据');
        subplot(2,1,1);
        plot(t_data, qdd_filtered, 'b-', 'LineWidth', 0.8);
        grid on; xlabel('时间 (s)'); ylabel('角加速度 (rad/s^2)');
        title('截取后数据：角加速度');
        subplot(2,1,2);
        if opts.tau_is_current
            plot(t_data, tau_real/K_tau, 'r-', 'LineWidth', 0.8);
            ylabel('电流 (A)');
        else
            plot(t_data, tau_real, 'r-', 'LineWidth', 0.8);
            ylabel('力矩 (N\cdot m)');
        end
        grid on; xlabel('时间 (s)');
        title('截取后数据：电流或力矩');

        % 惯量辨识：横坐标角加速度，纵坐标力矩（段平均点 + 拟合线）
        figure('Name', '惯量辨识');
        plot(x_fit, y_fit, 'b.', 'MarkerSize', 10);
        hold on;
        qdd_plot = linspace(min(x_fit), max(x_fit), 100);
        tau_plot = I_a_estimated * qdd_plot;
        plot(qdd_plot, tau_plot, 'r-', 'LineWidth', 2);
        grid on;
        xlabel('角加速度 (rad/s^2)');
        ylabel('力矩 (N\cdot m)');
        title(sprintf('惯量辨识: I_a = %.6f kg\\cdot m^2, R^2 = %.4f', I_a_estimated, R2_Ia));
        legend('匀加/匀减速段平均', '拟合 I_a \\times qdd', 'Location', 'best');

        % 加速度拟合图：上=qdd 时序，下=段平均点 + 拟合线
        figure('Name', '加速度拟合图');
        subplot(2,1,1);
        plot(t_data, qdd_display, 'b-', 'LineWidth', 1);
        grid on;
        xlabel('时间 (s)');
        ylabel('角加速度 q̈ (rad/s²)');
        title('角加速度轨迹（拟合基于匀加/匀减速段整段平均，见下图）');
        
        subplot(2,1,2);
        plot(x_fit, y_fit, 'b.', 'MarkerSize', 10);
        hold on;
        qdd_plot = linspace(min(x_fit), max(x_fit), 100);
        tau_plot = I_a_estimated * qdd_plot;  % 拟合：I_a * qdd
        plot(qdd_plot, tau_plot, 'r-', 'LineWidth', 2);
        grid on;
        xlabel('角加速度 mean(qdd) (rad/s^2)，匀加/匀减速段平均');
        ylabel('净力矩 mean(tau_net) (N·m)，匀加/匀减速段平均');
        title(sprintf('惯量拟合: I_a = %.6f kg\\cdot m^2, R^2 = %.4f（%d 个匀加/匀减速段）', I_a_estimated, R2_Ia, n_seg));
        legend('匀加/匀减速段平均 (mean(qdd), mean(\\tau_{net}))', '拟合 I_a\\times qdd', 'Location', 'best');
        
        % 整体拟合图：实际力矩 vs 整体模型力矩（I_a×q̈+τ_friction），图中屏蔽过大点
        tau_model_full = I_a_estimated * qdd_filtered + tau_friction;
        tau_model_display = tau_model_full;
        tau_model_display(idx_exclude_display) = NaN;
        figure('Name', '整体拟合图');
        subplot(2,1,1);
        plot(t_data, tau_real_display, 'b-', 'LineWidth', 0.8); hold on;
        plot(t_data, tau_model_display, 'r-', 'LineWidth', 0.8);
        grid on; xlabel('时间 (s)'); ylabel('力矩 (N\\cdot m)');
        title('整体拟合：实际力矩 vs 模型力矩（已屏蔽 |q̈|/|τ| 过大）');
        legend('实际力矩 \\tau_{real}', '模型力矩 I_a\\ddot{q}+\\tau_f', 'Location', 'best');
        subplot(2,1,2);
        plot(tau_real_display, tau_model_display, 'b.', 'MarkerSize', 4);
        hold on;
        ax = axis; lim = [min(ax(1),ax(3)), max(ax(2),ax(4))];
        plot(lim, lim, 'r--', 'LineWidth', 1);
        grid on; xlabel('实际力矩 \\tau_{real} (N\\cdot m)'); ylabel('模型力矩 (N\\cdot m)');
        title('整体拟合散点（斜线为理想一致，已屏蔽过大点）');
        legend('实测', '45° 理想线', 'Location', 'best');
    end

    % 诊断图：判断净力矩是否主要由惯量主导（若 τ_net 与 q̈ 形状相似则惯量主导）
    if opts.DoPlotDiagnostic
        figure('Name', '惯量辨识诊断：τ_real, τ_friction, τ_net, q̈');
        subplot(5,1,1);
        plot(t_data, tau_real, 'b-', 'LineWidth', 0.8);
        grid on; ylabel('\\tau_{real}'); title('实际力矩');
        subplot(5,1,2);
        plot(t_data, tau_friction, 'r-', 'LineWidth', 0.8);
        grid on; ylabel('\\tau_{friction}'); title('摩擦补偿');
        subplot(5,1,3);
        plot(t_data, tau_net, 'k-', 'LineWidth', 0.8);
        grid on; ylabel('\\tau_{net}'); title('净力矩');
        subplot(5,1,4);
        plot(t_data, qdd_filtered, 'g-', 'LineWidth', 0.8);
        grid on; ylabel('q̈'); title('角加速度');
        % 步骤③ 时间序列对比：若只是力矩延迟，τ_net 与 I_a×q̈ 形状相近但错位一两个采样
        subplot(5,1,5);
        plot(t_data, tau_net, 'k-', 'LineWidth', 0.8); hold on;
        plot(t_data, I_a_estimated * qdd_filtered, 'b--', 'LineWidth', 0.8);
        grid on; xlabel('时间 (s)'); ylabel('N\\cdot m');
        title('时间对齐检查：\\tau_{net} vs I_a\\times\\ddot{q}（若仅延迟则形状相近但错位）');
        legend('\\tau_{net}', 'I_a\\times\\ddot{q}', 'Location', 'best');
    end
    
else
    fprintf('  未得到有效 I_a（脉冲窗口点不足或符号检查未通过）。\n');
end

fprintf('\n===== 惯量辨识完成 =====\n');

end
