function Result = identify_friction_from_trajectory(varargin)
% identify_friction_from_trajectory  基于 Python 上位机控制电机速度的 CSV 进行摩擦力辨识
%
% 仅支持 Python 上位机导出格式（6 列或 7 列，含 direction 等文本列）：
%   6 列：timestamp_s, direction, target_speed_rpm, actual_speed_rpm, phase_current_A, temp_degC
%   7 列：多一列 command_current_A（在 target_speed_rpm 后）；辨识用电流为 phase_current_A（实际电流）
%
% 输入（名值对）:
%   data_file / data_files - 单文件路径或 cell 多文件（按顺序拼接）
%   K_tau                 - 力矩系数 N·m/A，默认 0.5
%   stable_start, stable_end - 稳定段去掉首尾点数，默认 100, 50
%   EstimateTauS          - 是否估计静摩擦，默认 true
%   DoPlot                - 是否绘图，默认 true
%   UseConstantVelocitySegments - 是否先提匀速段再每段取平均，默认 true
%   average_band          - 未使用；实际为 ≤22rpm 用 150、>22rpm 用 260
%   variance_threshold    - 备用；匀速段实际用：≤22rpm=0.1rpm，>22rpm 用 VarianceSpeedPct%%×|v|
%   VarianceSpeedPct     - 匀速段判定：速度标准差=速度的百分之几（误差），默认 10
%   variance_threshold_speed_scale - 未使用（由分段规则替代）
%   min_speed_rads        - 用于 τ_c、b 的匀速段需 |mean(qd)|>=此值，排除近零长段，默认 0.05
%   vel_change_threshold  - 匀速段切分速度阈值 rad/s，默认 0.01
%   min_seg_len           - 匀速段最少点数，少于此不参与取平均，默认 1000
%   KeepAllSegments       - true=不按设定档位合并，力只根据识别到的速度（每段一点），默认 true
%   MaxRpm                - 最大转速 rpm，用于生成档位表（与 scan_speed_levels_mit 一致），默认 192
%   GearRpmBand           - 每档在设定值 ±GearRpmBand rpm 内寻找匀速段求平均，默认 10
%   FrictionModel         - 摩擦模型：'coulomb_viscous' | 'tanh_viscous' | 'stribeck_viscous'
%       stribeck_viscous: τ_f=(τ_c+(τ_s-τ_c)e^{-(|v|/v_s)^2})sign(v)+bv+τ_0
%
% 输出: Result.tau_c, .tau_c_pos, .tau_c_neg, .b, .tau_s, .RMSE, .R2, .nUsed
%       tanh 模型时额外: Result.mu_s, .v_act, .mu_d
%       stribeck 模型时额外: Result.tau_s, .tau_c, .v_s, .tau0

p = inputParser;
addParameter(p, 'data_file', '', @ischar);
addParameter(p, 'data_files', {}, @(x) iscell(x) || (isstring(x) && isvector(x)));
addParameter(p, 'K_tau', 0.5, @isnumeric);
addParameter(p, 'stable_start', 100, @isnumeric);
addParameter(p, 'stable_end', 50, @isnumeric);
addParameter(p, 'EstimateTauS', true, @islogical);
addParameter(p, 'DoPlot', true, @islogical);
addParameter(p, 'UseConstantVelocitySegments', true, @islogical);
addParameter(p, 'average_band', 300, @isnumeric);  % 保留兼容；实际用 ab_low=150/ab_high=260
addParameter(p, 'variance_threshold', 0.001, @isnumeric);  % 仅作备用
addParameter(p, 'VarianceSpeedPct', 10, @isnumeric);  % 匀速段判定：速度标准差=速度的此百分比（误差），默认 10
addParameter(p, 'variance_threshold_speed_scale', 0.05, @isnumeric);  % 未使用
addParameter(p, 'min_speed_rads', 0.05, @isnumeric);
addParameter(p, 'vel_change_threshold', 0.01, @isnumeric);
addParameter(p, 'min_seg_len', 1000, @isnumeric);
addParameter(p, 'KeepAllSegments', true, @islogical);   % true=不按设定档位合并，力只根据识别到的速度
addParameter(p, 'MaxRpm', 192, @isnumeric);  % 与 scan_speed_levels_mit 一致时的最大转速 rpm
addParameter(p, 'GearRpmBand', 10, @isnumeric);  % 档位 ±GearRpmBand rpm 内取匀速段求平均
addParameter(p, 'FrictionModel', 'coulomb_viscous', @ischar);  % 'coulomb_viscous' | 'tanh_viscous' | 'stribeck_viscous'
parse(p, varargin{:});
opts = p.Results;
if isstring(opts.data_files), opts.data_files = cellstr(opts.data_files); end
if ~isempty(opts.data_files), opts.data_file = ''; end
if isempty(opts.data_file) && isempty(opts.data_files)
    opts.data_file = fullfile(get_data_dir('friction_iden'), 'friction_scan.csv');
end

% 路径：robot_algorithm（identify_joint_friction）、utility_function（read_python_upper_scan_csv, get_data_dir）
cur = fileparts(mfilename('fullpath'));
rp = fullfile(cur, '..', '..', 'robot_algorithm');
if exist(rp, 'dir') && isempty(which('identify_joint_friction'))
    addpath(genpath(rp));
end
up = fullfile(cur, '..', '..', 'utility_function');
if exist(up, 'dir') && isempty(which('read_python_upper_scan_csv'))
    addpath(up);
end

file_list = opts.data_files;
if isempty(file_list), file_list = {opts.data_file}; end

fprintf('===== 摩擦力辨识（Python 上位机 6/7 列格式）=====\n');
fprintf('数据: %s\n', strjoin(file_list, ', '));
fprintf('K_tau: %.4f N·m/A\n', opts.K_tau);

%% 读取：时间、电流、速度 rad/s（read_python_upper_scan_csv 内已做 rpm→rad/s，此处不再转换）
data = read_python_upper_scan_csv(file_list);
fprintf('  已读共 %d 行\n', size(data, 1));

if size(data, 1) < 2
    error('有效数据不足 2 行，请检查 CSV 格式（6/7 列：timestamp_s, direction, ..., actual_speed_rpm, phase_current_A, ...）');
end

t_data = data(:, 1);
i_act = data(:, 2);
qd_act = data(:, 3);  % 已是 rad/s，带正负（由 read_python_upper_scan_csv 按 direction 赋号并转换）
% 力矩与速度同向：若 CSV 中 phase_current 为绝对值，则按速度符号取号，使 τ = K_τ×i 在 qd<0 时为负
i_act = sign(qd_act) .* abs(i_act);
% 读取后可选降采样，downsample_step=1 表示不降采样
downsample_step = 1;
idx_ds = 1 : downsample_step : length(t_data);
t_data = t_data(idx_ds);
i_act = i_act(idx_ds);
qd_act = qd_act(idx_ds);
if downsample_step > 1
    fprintf('数据: 降采样后 %d 行 (隔 %d 取 1), 时长 %.2f s\n', length(t_data), downsample_step, t_data(end));
else
    fprintf('数据: 不降采样 %d 行, 时长 %.2f s\n', length(t_data), t_data(end));
end

%% 稳定段
n = length(qd_act);
st = min(opts.stable_start, floor(n/4));
en = min(opts.stable_end, floor(n/4));
t_stable = t_data((st+1):(n-en));
qd_stable = qd_act((st+1):(n-en));
i_stable = i_act((st+1):(n-en));

%% 可选：匀速段（局部方差 < 阈值；正向与负向速度用同一套规则，均按 |qd|）
% 窗口：|qd|≤22rpm 用 150、否则 100（窗口小一些便于识别短档，保留原始等级数）
% 阈值：速度标准差 ≤ 速度×VarianceSpeedPct%（默认 10% 误差）
ab_low  = 150;   % 22 rpm 以下窗口半宽（正负同）
ab_high = 100;   % 22 rpm 以上窗口半宽（正负同）
variance_speed_pct = opts.VarianceSpeedPct / 100;  % 如 10 → 0.10
min_spd = opts.min_speed_rads;
low_speed_pct = variance_speed_pct;   % 低/高速统一用 VarianceSpeedPct 作为匀速判定误差
rpm22_rads = 22 * (2*pi/60);
ab_max = max(ab_low, ab_high);  % 循环范围需覆盖两种窗口
if opts.UseConstantVelocitySegments && length(qd_stable) >= (2*ab_max+1)
    qd = qd_stable(:);
    nq = length(qd);
    idx_uniform_v = [];
    for i = (1+ab_max) : (nq-ab_max)
        % 先用小窗口判速度，决定用哪档窗口
        ave_v_pre = mean(qd(i-ab_low:i+ab_low));
        if abs(ave_v_pre) <= rpm22_rads
            ab = ab_low;   % 22 rpm 以下用 150
        else
            ab = ab_high;  % 其余用 100
        end
        v_band = qd(i-ab : i+ab);
        ave_v = mean(v_band);
        check_value = sum((v_band - ave_v).^2) / (2*ab+1);
        % 方差阈值：标准差 = 速度×VarianceSpeedPct%（默认 10% 误差）
        abs_qd = abs(ave_v);
        vth_eff = (low_speed_pct * abs_qd)^2;
        if check_value < vth_eff
            idx_uniform_v = [idx_uniform_v; i];
        end
    end
    t_stable = t_stable(idx_uniform_v);
    qd_stable = qd(idx_uniform_v);
    i_stable = i_stable(idx_uniform_v);
    % 按连续段合并：每段保留一点 (mean(qd), mean(τ))
    gap = diff(idx_uniform_v) > 1;
    seg_end = [find(gap); length(idx_uniform_v)];
    seg_start = [1; find(gap) + 1];
    qd_avg = [];
    tau_avg = [];
    t_avg = [];
    for j = 1:length(seg_end)
        idx_seg = seg_start(j):seg_end(j);
        mqd = mean(qd_stable(idx_seg));
        % 仅保留 |qd|>=min_speed_rads 的段用于 τ_c、b，排除长时间近零段
        if abs(mqd) >= min_spd
            qd_avg = [qd_avg; mqd];
            tau_avg = [tau_avg; mean(i_stable(idx_seg))];
            t_avg = [t_avg; mean(t_stable(idx_seg))];
        end
    end
    if isempty(qd_avg)
        % 无段满足 |qd|>=min_speed，回退为保留全部段（含近零）
        for j = 1:length(seg_end)
            idx_seg = seg_start(j):seg_end(j);
            qd_avg = [qd_avg; mean(qd_stable(idx_seg))];
            tau_avg = [tau_avg; mean(i_stable(idx_seg))];
            t_avg = [t_avg; mean(t_stable(idx_seg))];
        end
        qd_stable = qd_avg; i_stable = tau_avg; t_stable = t_avg;
        fprintf('  匀速段: 窗口 ≤22rpm=%d/否则=%d 点, 速度误差 %.0f%%×|v|, |qd|>=%.3f → %d 个点\n', 2*ab_low+1, 2*ab_high+1, opts.VarianceSpeedPct, min_spd, length(qd_stable));
    else
        qd_stable = qd_avg;
        i_stable = tau_avg;
        t_stable = t_avg;
        fprintf('  匀速段: 窗口 ≤22rpm=%d/否则=%d 点, 速度误差 %.0f%%×|v|, |qd|>=%.3f rad/s → %d 个速度档点\n', 2*ab_low+1, 2*ab_high+1, opts.VarianceSpeedPct, min_spd, length(qd_stable));
    end
end

%% 力只根据识别到的速度：KeepAllSegments=true 时每段一点（速度=该段均值），不按设定档位合并
% KeepAllSegments=false 时：按档位 ±GearRpmBand 合并，合并后速度仍用识别到的均值，不用设定值
qd_setpoints_pos = build_speed_levels_mit(opts.MaxRpm);  % 正向档位 rad/s
all_setpoints_rads = sort(unique([-qd_setpoints_pos; qd_setpoints_pos]));
rads2rpm = 60/(2*pi);
band_rpm = opts.GearRpmBand;  % ±band_rpm 内取匀速段

if opts.KeepAllSegments
    % 不合并：直接用匀速段结果，保留所有识别到的等级
    t_stable = (1:length(qd_stable))';
    fprintf('  保留每段一点，共 %d 个速度档点（未按档位表合并）\n', length(qd_stable));
else
    % 每档：在 设定转速 ± band_rpm 内找匀速段，对该档的速度和电流分别求平均
    qd_stable_rpm = qd_stable(:) * rads2rpm;  % 当前各段平均速度 (rpm)
    qd_per_gear = nan(length(all_setpoints_rads), 1);
    i_per_gear = nan(length(all_setpoints_rads), 1);
    for k = 1:length(all_setpoints_rads)
        setpoint_rpm = all_setpoints_rads(k) * rads2rpm;
        in_band = abs(qd_stable_rpm - setpoint_rpm) <= band_rpm;
        if any(in_band)
            qd_per_gear(k) = mean(qd_stable(in_band));   % 速度用 rad/s 平均
            i_per_gear(k)  = mean(i_stable(in_band));
        end
    end
    valid = ~isnan(qd_per_gear);
    qd_stable = qd_per_gear(valid);
    i_stable = i_per_gear(valid);
    t_stable = (1:length(qd_stable))';
    fprintf('  按档位合并：每档 ±%d rpm 内匀速段求平均 → %d 个点（max_rpm=%.0f）\n', band_rpm, sum(valid), opts.MaxRpm);
end

%% 绘制截取后的速度、电流（横轴：角速度，摩擦力辨识）
if opts.DoPlot
    figure('Name', '截取后数据：速度与电流');
    subplot(2,1,1);
    plot(qd_stable, (1:length(qd_stable))', 'b.-', 'MarkerSize', 6);
    grid on; xlabel('角速度 (rad/s)'); ylabel('档位序号'); title('截取后速度（每档一点）');
    subplot(2,1,2);
    plot(qd_stable, i_stable, 'r.-', 'MarkerSize', 6);
    grid on; xlabel('角速度 (rad/s)'); ylabel('电流 (A)'); title('截取后电流（每档一点）');
end

%% 辨识：按所选摩擦模型拟合（速度与力均用识别到的值，不用设定档位）
tau_stable = opts.K_tau * i_stable;  % 力矩 (N·m)
if strcmpi(opts.FrictionModel, 'stribeck_viscous')
    % Stribeck + 粘滞 + 偏置: τ_f = (τ_c+(τ_s-τ_c)e^{-(|v|/v_s)^2})sign(v) + b*v + τ_0
    % 参数 p = [tau_c, delta, v_s, b, tau0], tau_s = tau_c + delta, delta>=0
    qd = qd_stable(:);
    tau = tau_stable(:);
    n_used = length(qd);
    tau_c0 = 0.5 * max(abs(tau));
    if tau_c0 < 1e-6, tau_c0 = 0.1; end
    delta0 = 0.3 * max(abs(tau));
    v_s0 = 0.5;
    b0 = 0.01;
    tau00 = 0;
    x0 = [tau_c0; delta0; v_s0; b0; tau00];
    lb = [0; 0; 0.001; 0; -2];
    ub = [20; 20; 5; 5; 2];
    opt = optimoptions('lsqnonlin', 'Display', 'off');
    stribeck_resid = @(p) tau - ((p(1) + p(2)*exp(-(abs(qd)/max(p(3),1e-6)).^2)).*sign(qd) + p(4)*qd + p(5));
    x_opt = lsqnonlin(stribeck_resid, x0, lb, ub, opt);
    tau_c = x_opt(1);
    delta = x_opt(2);
    tau_s_stribeck = tau_c + delta;
    v_s = x_opt(3);
    b_stribeck = x_opt(4);
    tau0 = x_opt(5);
    tau_pred = (tau_c + delta*exp(-(abs(qd)/v_s).^2)).*sign(qd) + b_stribeck*qd + tau0;
    ss_res = sum((tau - tau_pred).^2);
    ss_tot = sum((tau - mean(tau)).^2);
    R2 = 1 - ss_res / max(ss_tot, 1e-10);
    RMSE = sqrt(mean((tau - tau_pred).^2));
    Result = struct();
    Result.FrictionModel = 'stribeck_viscous';
    Result.tau_s = tau_s_stribeck;
    Result.tau_c = tau_c;
    Result.v_s = v_s;
    Result.b = b_stribeck;
    Result.tau0 = tau0;
    Result.tau_c_pos = tau_c;
    Result.tau_c_neg = -tau_c;
    Result.R2 = R2;
    Result.RMSE = RMSE;
    Result.nUsed = n_used;
    fprintf('\n===== 结果（Stribeck + 粘滞 + 偏置）=====\n');
    fprintf('τ_s = %.4f, τ_c = %.4f N·m, v_s = %.4f rad/s, b = %.4f, τ_0 = %.4f N·m\n', tau_s_stribeck, tau_c, v_s, b_stribeck, tau0);
    fprintf('R² = %.4f, RMSE = %.4f N·m, n = %d\n', R2, RMSE, n_used);
    if opts.DoPlot
        figure('Name', '摩擦辨识（Stribeck 模型）');
        plot(qd, tau, 'b.', 'MarkerSize', 8); hold on;
        qd_plot = linspace(min(qd), max(qd), 200);
        tau_plot = (tau_c + delta*exp(-(abs(qd_plot)/v_s).^2)).*sign(qd_plot) + b_stribeck*qd_plot + tau0;
        plot(qd_plot, tau_plot, 'r-', 'LineWidth', 1.5);
        grid on; xlabel('角速度 (rad/s)'); ylabel('力矩 (N\cdot m)');
        title(sprintf('Stribeck: \\tau_s=%.3f, \\tau_c=%.3f, v_s=%.3f, b=%.4f, \\tau_0=%.4f, R^2=%.4f', tau_s_stribeck, tau_c, v_s, b_stribeck, tau0, R2));
        legend('实测', '拟合', 'Location', 'best');
    end
elseif strcmpi(opts.FrictionModel, 'tanh_viscous')
    % 模型 τ = μs·tanh(v/vact) + μd·v
    qd = qd_stable(:);
    tau = tau_stable(:);
    n_used = length(qd);
    % 初值：μs 约等于高速段力矩幅值，v_act 小正数，μd 粘滞
    mu_s0 = max(abs(tau));
    if mu_s0 < 1e-6, mu_s0 = 0.1; end
    v_act0 = max(0.05, 0.1 * (max(abs(qd)) + 1e-6));
    mu_d0 = 0.01;
    x0 = [mu_s0; v_act0; mu_d0];
    lb = [1e-6; 1e-6; 0];
    ub = [inf; inf; inf];
    opt = optimoptions('lsqnonlin', 'Display', 'off');
    fun = @(x) tau - (x(1)*tanh(qd/max(x(2),1e-8)) + x(3)*qd);
    x_opt = lsqnonlin(fun, x0, lb, ub, opt);
    mu_s = x_opt(1); v_act = x_opt(2); mu_d = x_opt(3);
    tau_pred = mu_s*tanh(qd/v_act) + mu_d*qd;
    ss_res = sum((tau - tau_pred).^2);
    ss_tot = sum((tau - mean(tau)).^2);
    R2 = 1 - ss_res/max(ss_tot, 1e-10);
    RMSE = sqrt(mean((tau - tau_pred).^2));
    Result = struct();
    Result.FrictionModel = 'tanh_viscous';
    Result.mu_s = mu_s;
    Result.v_act = v_act;
    Result.mu_d = mu_d;
    Result.tau_c = mu_s;
    Result.tau_c_pos = mu_s;
    Result.tau_c_neg = -mu_s;
    Result.b = mu_d;
    Result.R2 = R2;
    Result.RMSE = RMSE;
    Result.nUsed = n_used;
    Result.tau_s = NaN;  % 静摩擦由外部电流扫描得到时可再赋值
    fprintf('\n===== 结果（tanh 模型 μs·tanh(v/vact)+μd·v）=====\n');
    fprintf('μ_s = %.4f N·m, v_act = %.4f rad/s, μ_d = %.4f N·m·s/rad\n', mu_s, v_act, mu_d);
    if opts.DoPlot
        figure('Name', '摩擦辨识（tanh 模型）');
        plot(qd, tau, 'b.', 'MarkerSize', 8); hold on;
        qd_plot = linspace(min(qd), max(qd), 200);
        tau_plot = mu_s*tanh(qd_plot/v_act) + mu_d*qd_plot;
        plot(qd_plot, tau_plot, 'r-', 'LineWidth', 1.5);
        grid on; xlabel('角速度 (rad/s)'); ylabel('力矩 (N\cdot m)');
        title(sprintf('tanh 模型: \\mu_s=%.3f, v_{act}=%.3f, \\mu_d=%.4f, R^2=%.4f', mu_s, v_act, mu_d, R2));
        legend('实测', '拟合', 'Location', 'best');
    end
else
    % 库伦+粘滞：τ_c·sign(v)+b·v（或不对称 τ_c_pos/τ_c_neg）
    Result = identify_joint_friction(qd_stable, i_stable, opts.K_tau, ...
        'EstimateTauS', opts.EstimateTauS, 'DoPlot', opts.DoPlot);
    Result.FrictionModel = 'coulomb_viscous';
    fprintf('\n===== 结果 =====\n');
    fprintf('τ_c = %.4f (τ_c+ = %.4f, τ_c- = %.4f) N·m, b = %.4f N·m·s/rad\n', ...
        Result.tau_c, Result.tau_c_pos, Result.tau_c_neg, Result.b);
    if isfield(Result,'tau_s') && ~isnan(Result.tau_s), fprintf('τ_s = %.4f N·m\n', Result.tau_s); end
    fprintf('R² = %.4f, RMSE = %.4f N·m, n = %d\n', Result.R2, Result.RMSE, Result.nUsed);
end

end

%% 与 scan_speed_levels_mit 一致的速度档位：低速精细表 + 1~max_rpm 等分 10 档
function qd_rads = build_speed_levels_mit(max_rpm)
% 输出：正向档位角速度 (rad/s)，与 jason_mit/scan_speed_levels_mit.py 的 build_speed_levels 一致
LEVEL_RPMS_UP_TO_954 = [ ...
    0.095492967, 0.190985935, 0.286478902, 0.38197187, 0.477464837, ...
    0.572957805, 0.668450772, 0.76394374, 0.859436707, 0.954929675, ...
    1.050422642, 1.14591561, 1.241408577, 1.336901545, 1.432394512, ...
    1.52788748, 1.623380447, 1.718873415, 1.814366382, 1.90985935, ...
    2.005352317, 2.100845285, 2.196338252, 2.29183122, 2.387324187, ...
    2.482817155, 2.578310122, 2.67380309, 2.769296057, 2.864789025, ...
    3.819718699, 4.774648374, 5.729578049, 6.684507724, 7.639437399, ...
    8.594367074, 9.549296748 ...
];
low = LEVEL_RPMS_UP_TO_954(LEVEL_RPMS_UP_TO_954 <= max_rpm);
high = 1.0 + (max_rpm - 1.0) * (0:9) / 9.0;   % 1 ~ max_rpm 等分 10 档
rpms = sort(unique([low(:); high(:)]));
qd_rads = rpms * (2*pi/60);   % rpm -> rad/s
qd_rads = qd_rads(:);
end
