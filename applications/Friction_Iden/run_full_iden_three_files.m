% run_full_iden_three_files  三文件一次完成：静摩擦 τ_s、库伦/粘滞 τ_c/b、惯量 I_a
%
% 对应 Python 上位机三种扫描：
%   文件1 电流扫描 CSV   → τ_s（静摩擦）
%   文件2 速度扫描 CSV   → τ_c、b（库伦、粘滞）
%   文件3 加速度扫描 CSV → I_a（转动惯量，需先有 τ_c、b）
%
% 改 file_static、file_velocity、file_accel、K_tau、FrictionModel、Fs 后直接运行。
% 加速度扫描第二列含义：file_accel_is_torque = true 表示力矩，false 表示电流；不设或 [] 则按文件名（含 scan_torque_mit_over_rapoo 则为力矩）自动判断。
% FrictionModel: 'coulomb_viscous'（τ_c·sign(v)+b·v）或 'tanh_viscous'（μs·tanh(v/vact)+μd·v）
% 采样频率 Fs=200 Hz，图中横坐标 行数/Fs = 时间 (s)。

clear; clc; close all;

% 摩擦模型选择
FrictionModel = 'tanh_viscous';   % 'coulomb_viscous' | 'tanh_viscous' | 'stribeck_viscous'

% 确保 utility_function 与当前目录在路径
cur = fileparts(mfilename('fullpath'));
addpath(cur);
up = fullfile(cur, '..', '..', 'utility_function');
if exist(up, 'dir') && isempty(which('read_python_upper_scan_csv'))
    addpath(up);
end

% dd = get_data_dir('test_data/泉智博_100-20-10 26_3_6');
% file_static   = fullfile(dd, 'scan_torque_serial_mit_20260309_192539.csv');    % 电流扫描（静摩擦）3
% file_velocity = fullfile(dd, 'scan_result_mit_20260306_154134.csv');      % 速度扫描（τ_c、b）
% file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260311_182442.csv');      % 加速度扫描（I_a），第二列为电流
% file_accel_is_torque = false;   % 上行为电流型文件；若为力矩型文件则设 true
% K_tau = 1.8;
% VarianceSpeedPct = 5;   % 匀速段判定：速度标准差≤速度的此百分比（可改，如 7、15）

% dd = get_data_dir('test_data/泉智博_PA81_12_18');
% file_static   = fullfile(dd, 'scan_torque_serial_mit_20260309_184507.csv');    % 电流扫描（静摩擦）3
% file_velocity = fullfile(dd, 'scan_result_mit_20260310_112034_v.csv');      % 速度扫描（τ_c、b）
% file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260311_181156.csv');      % 加速度扫描（I_a）
% K_tau = 1.75;
% VarianceSpeedPct = 7;   % 匀速段判定：速度标准差≤速度的此百分比（可改，如 7、15）

% dd = get_data_dir('test_data/泉智博_43_15_36');
% file_static   = fullfile(dd, 'scan_torque_serial_mit_20260309_155636.csv');    % 电流扫描（静摩擦）3
% file_velocity = fullfile(dd, 'scan_result_mit_20260309_150326.csv');      % 速度扫描（τ_c、b）
% file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260311_171910.csv');      % 加速度扫描（I_a）
% K_tau = 2.9;
% VarianceSpeedPct = 5;   % 匀速段判定：速度标准差≤速度的此百分比（可改，如 7、15）

% dd = get_data_dir('test_data/泉智博_43_10_40');
% file_static   = fullfile(dd, 'scan_torque_serial_mit_20260309_174657.csv');    % 电流扫描（静摩擦）3
% file_velocity = fullfile(dd, 'scan_result_mit_20260311_151710.csv');      % 速度扫描（τ_c、b）
% file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260311_171158.csv');      % 加速度扫描（I_a）
% K_tau = 4.1;
% VarianceSpeedPct = 5;   % 匀速段判定：速度标准差≤速度的此百分比（可改，如 7、15）

dd = get_data_dir('test_data/因克斯EC-A10020-P2-72V');
file_static   = fullfile(dd, 'scan_current_serial_20260310_164327.csv');    % 电流扫描（静摩擦）3
file_velocity = fullfile(dd, 'scan_result_20260310_170140.csv');      % 速度扫描（τ_c、b）
file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260311_160046.csv');      % 加速度扫描（I_a）
file_accel_is_torque = false;   % true=该文件第二列为力矩；false=电流；不设或[]=按文件名自动判断（含 scan_torque_mit_over_rapoo 则为力矩）
K_tau = 2.6;
VarianceSpeedPct = 5;   % 匀速段判定：速度标准差≤速度的此百分比（可改，如 7、15）

speed_still_rads = 0.05 * (2*pi/60);  % 静止判定阈值 0.05 rpm → rad/s（数据已是 rad/s）
stable_start = 100;
stable_end   = 20;
accel_threshold = 0;   % 惯量拟合：仅用 |q̈|>此值的点（rad/s²），0=用全部点；可改如 30、80
accel_exclude_above = 500;   % 剔除 |q̈|>此值的点（拟合与整体误差），Inf=不剔除
tau_exclude_above = 500;     % 剔除 |τ|>此值的点（N·m，拟合与整体误差），Inf=不剔除
Fs = 200;   % 采样频率 Hz；横坐标 行数/Fs = 时间 (s)

%% 0. 先看：各文件原始速度与加速度
fprintf('===== 0/3 原始速度与加速度预览 =====\n');
if exist(file_static, 'file')
    ds = read_python_upper_scan_csv(file_static);
else
    ds = zeros(0,3);
end
if exist(file_velocity, 'file')
    dv = read_python_upper_scan_csv(file_velocity);
else
    dv = zeros(0,3);
end
if exist(file_accel, 'file')
    da = read_python_upper_scan_csv(file_accel);
else
    da = zeros(0,3);
end
% 由速度数值微分得到加速度（不平滑）
qdd_s = accel_from_vel(ds); qdd_v = accel_from_vel(dv); qdd_a = accel_from_vel(da);

figure('Name', '原始速度与加速度预览（各文件）');
% 静摩擦文件：速度、加速度
subplot(3,2,1);
if ~isempty(ds)
    plot((1:size(ds,1))'/Fs, ds(:,3), 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角速度 (rad/s)');
title('静摩擦文件：速度');
subplot(3,2,2);
if ~isempty(ds)
    plot((1:length(qdd_s))'/Fs, qdd_s, 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角加速度 (rad/s^2)');
title('静摩擦文件：加速度');

% 速度扫描文件：速度、加速度
subplot(3,2,3);
if ~isempty(dv)
    plot((1:size(dv,1))'/Fs, dv(:,3), 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角速度 (rad/s)');
title('速度扫描文件：速度');
subplot(3,2,4);
if ~isempty(dv)
    plot((1:length(qdd_v))'/Fs, qdd_v, 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角加速度 (rad/s^2)');
title('速度扫描文件：加速度');

% 加速度扫描文件：速度、加速度
subplot(3,2,5);
if ~isempty(da)
    plot((1:size(da,1))'/Fs, da(:,3), 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角速度 (rad/s)');
title('加速度扫描文件：速度');
subplot(3,2,6);
if ~isempty(da)
    plot((1:length(qdd_a))'/Fs, qdd_a, 'b-', 'LineWidth', 0.8);
end
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('角加速度 (rad/s^2)');
title('加速度扫描文件：加速度');

%% 1. 静摩擦 τ_s（电流扫描：静止段 max|电流|×K_tau）
fprintf('===== 1/3 静摩擦文件 → τ_s =====\n');
data_static = read_python_upper_scan_csv(file_static);  % [t, current_A, speed_rad/s]
qd_list = data_static(:, 3);  % 已是 rad/s
cur_list = data_static(:, 2);
downsample_step = 10;
idx_ds = 1 : downsample_step : length(qd_list);
qd_list = qd_list(idx_ds);
cur_list = cur_list(idx_ds);
fprintf('  电流扫描降采样后 %d 点\n', length(qd_list));
idx = abs(qd_list) < speed_still_rads;
tau_s = NaN;
if any(idx)
    tau_s = K_tau * max(abs(cur_list(idx)));
    fprintf('  静止段 %d 点 → τ_s = %.4f N·m\n', sum(idx), tau_s);
else
    fprintf('  无静止段，τ_s = NaN\n');
end
% 静摩擦辨识后绘图：横坐标力矩，纵坐标速度
torque_list = K_tau * cur_list;
figure('Name', '静摩擦辨识数据');
plot(torque_list, qd_list, 'b.', 'MarkerSize', 4);
grid on; xlabel('力矩 (N\cdot m)'); ylabel('速度 (rad/s)');
title(sprintf('静摩擦辨识 (\\tau_s = %.4f N\\cdot m)', tau_s));

%% 2. 库伦/粘滞 τ_c、b（速度扫描）
fprintf('\n===== 2/3 速度扫描文件 → τ_c、b =====\n');
if ~exist(file_velocity, 'file'), error('速度扫描文件不存在: %s', file_velocity); end
Result_friction = identify_friction_from_trajectory(...
    'data_file', file_velocity, 'K_tau', K_tau, ...
    'stable_start', stable_start, 'stable_end', stable_end, ...
    'VarianceSpeedPct', VarianceSpeedPct, ...
    'EstimateTauS', false, 'DoPlot', true, 'FrictionModel', FrictionModel, ...
    'KeepAllSegments', true);   % 保留每段一点，便于 3 rad/s 以上识别到多档
Result_friction.tau_s = tau_s;

%% 3. 转动惯量 I_a（加速度扫描，用步骤 1、2 辨识的 τ_c_pos、τ_c_neg、b 做摩擦补偿）
% 若 R² 很低可试：'VelocityFilter','lowpass'；'TauShiftSamples',1；'accel_threshold',80；'DoPlotDiagnostic',true
% 三步骤误差来源检查：置为 true 时跑 3 次（基线 / τ 平移 1 帧 / 仅 |q̈|>80）并打印 R² 对比
run_three_step_check = true;
% 加速度轨迹是否为「分段正弦」（0.1~2 Hz、10 段×6 s、A=0.5 rad）生成；是则用分段正弦惯量辨识，精度更高
use_segmented_sinusoidal_inertia = true;
fprintf('\n===== 3/3 加速度扫描文件 → I_a =====\n');
if ~exist(file_accel, 'file')
    fprintf('  未提供加速度文件，跳过惯量辨识。\n');
    I_a = NaN; I_a_R2 = NaN;
else
    % 三步骤若显示 ③ 明显优于 ①，说明小加速度段噪声/摩擦残差主导；主辨识可用仅大加速度（use_large_accel_only=true）
    use_large_accel_only = true;   % true：仅 |q̈|>accel_threshold 拟合（R² 高时采用）；false：全部点
    % 加速度文件第二列：力矩则不再×K_tau，电流则×K_tau
    if exist('file_accel_is_torque', 'var') && ~isempty(file_accel_is_torque)
        tau_is_current_accel = ~logical(file_accel_is_torque);
    else
        [~, fname_accel] = fileparts(file_accel);
        tau_is_current_accel = ~contains(fname_accel, 'scan_torque_mit_over_rapoo');
    end
    base_opts = {'TauCPos', Result_friction.tau_c_pos, 'TauCNeg', Result_friction.tau_c_neg, ...
        'tau_is_current', tau_is_current_accel, 'VelocityUnit', 'rpm', 'DataFormat', 'python_upper', 'DoPlot', ~run_three_step_check, ...
        'VelocityFilter', 'lowpass', 'LowpassCutoffHz', 20, ...
        'AccelExcludeAbove', accel_exclude_above, 'TauExcludeAbove', tau_exclude_above};   % 剔除 |q̈|、|τ| 过大的点
    if strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
        base_opts = [base_opts, {'TauS', Result_friction.tau_s, 'TauC', Result_friction.tau_c, 'Vs', Result_friction.v_s, 'Tau0', Result_friction.tau0}];
    end
    if use_segmented_sinusoidal_inertia
        % 分段正弦轨迹（0.1~2 Hz、10 段×6 s）：每段拟合 τ_net=J*qdd(+c)，按 R² 加权得 I_a
        seg_opts = {'TauCPos', Result_friction.tau_c_pos, 'TauCNeg', Result_friction.tau_c_neg, ...
            'tau_is_current', tau_is_current_accel, 'DataFormat', 'python_upper', ...
            'SegmentDuration', 6, 'NumSegments', 10, 'MiddleFrac', 0.6, ...
            'R2Min', 0, 'AccelMeanMin', 0, 'FitWithBias', true, 'JMethod', 'weighted_mean', ...
            'VelocityFilter', 'lowpass', 'LowpassCutoffHz', 12};
        if strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
            seg_opts = [seg_opts, {'TauS', Result_friction.tau_s, 'TauC', Result_friction.tau_c, ...
                'Vs', Result_friction.v_s, 'Tau0', Result_friction.tau0}];
        end
        Result_inertia = identify_inertia_segmented_sinusoidal(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, seg_opts{:});
    elseif run_three_step_check
        fprintf('  ----- 三步骤误差来源检查（R² 对比）-----\n');
        R0 = identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, base_opts{:});
        fprintf('  ① 基线: R² = %.4f, I_a = %.6f\n', R0.I_a_R2, R0.I_a);
        R1 = identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, base_opts{:}, 'TauShiftSamples', 1);
        fprintf('  ② τ 平移 1 帧: R² = %.4f, I_a = %.6f\n', R1.I_a_R2, R1.I_a);
        R2 = identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, base_opts{:}, 'accel_threshold', accel_threshold);
        fprintf('  ③ 仅 |q̈|>%.4g: R² = %.4f, I_a = %.6f\n', accel_threshold, R2.I_a_R2, R2.I_a);
        fprintf('  ----- 若 ② 明显高于 ① 多为力矩延迟；若 ③ 明显高于 ① 多为小加速度段噪声/摩擦残差 -----\n');
        % 若 ③ 明显更好则采用 ③ 作为最终结果，否则用 ①
        if R2.I_a_R2 > R0.I_a_R2 + 0.1
            Result_inertia = R2;
            fprintf('  采用 ③（仅大加速度）作为本次惯量结果。\n');
        else
            Result_inertia = R0;
        end
        % 三步骤检查时前几次 DoPlot=false，补画一次“加速度拟合图”“整体拟合图”供查看与保存
        if R2.I_a_R2 > R0.I_a_R2 + 0.1
            identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, ...
                base_opts{:}, 'DoPlot', true, 'accel_threshold', accel_threshold);
        else
            identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, ...
                base_opts{:}, 'DoPlot', true);
        end
    else
        if use_segmented_sinusoidal_inertia
            seg_opts = {'TauCPos', Result_friction.tau_c_pos, 'TauCNeg', Result_friction.tau_c_neg, ...
                'tau_is_current', tau_is_current_accel, 'DataFormat', 'python_upper', ...
                'SegmentDuration', 6, 'NumSegments', 10, 'MiddleFrac', 0.6, ...
                'R2Min', 0, 'AccelMeanMin', 0, 'FitWithBias', true, 'JMethod', 'weighted_mean', ...
                'VelocityFilter', 'lowpass', 'LowpassCutoffHz', 12};
            if strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
                seg_opts = [seg_opts, {'TauS', Result_friction.tau_s, 'TauC', Result_friction.tau_c, 'Vs', Result_friction.v_s, 'Tau0', Result_friction.tau0}];
            end
            Result_inertia = identify_inertia_segmented_sinusoidal(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, seg_opts{:});
        elseif use_large_accel_only
            Result_inertia = identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, base_opts{:}, 'accel_threshold', accel_threshold);
        else
            Result_inertia = identify_inertia_from_data(file_accel, K_tau, Result_friction.tau_c, Result_friction.b, base_opts{:});
        end
    end
    I_a = Result_inertia.I_a;
    I_a_R2 = Result_inertia.I_a_R2;
end
% 整体拟合误差用同一数据/范围：惯量来自加速度文件时在加速度数据上算误差，否则在速度扫描上算
overall_data_file = file_velocity;
if exist('use_segmented_sinusoidal_inertia', 'var') && use_segmented_sinusoidal_inertia && exist(file_accel, 'file') && ~isnan(I_a)
    overall_data_file = file_accel;
    fprintf('\n  整体拟合误差将基于加速度数据（与惯量辨识统一数据/范围）。\n');
end

%% 汇总
fprintf('\n========== 三文件辨识结果 ==========\n');
fprintf('摩擦模型   %s\n', Result_friction.FrictionModel);
fprintf('静摩擦     τ_s = %.4f N·m   （来自电流扫描）\n', tau_s);
if strcmpi(Result_friction.FrictionModel, 'tanh_viscous')
    fprintf('tanh 参数   μ_s = %.4f N·m, v_act = %.4f rad/s, μ_d = %.4f N·m·s/rad\n', ...
        Result_friction.mu_s, Result_friction.v_act, Result_friction.mu_d);
elseif strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
    fprintf('Stribeck    τ_s = %.4f, τ_c = %.4f N·m, v_s = %.4f rad/s, b = %.4f, τ_0 = %.4f N·m\n', ...
        Result_friction.tau_s, Result_friction.tau_c, Result_friction.v_s, Result_friction.b, Result_friction.tau0);
else
    fprintf('库伦摩擦   τ_c = %.4f N·m   （τ_c_pos = %.4f, τ_c_neg = %.4f）\n', Result_friction.tau_c, Result_friction.tau_c_pos, Result_friction.tau_c_neg);
    fprintf('粘滞摩擦   b   = %.4f N·m·s/rad\n', Result_friction.b);
end
if ~isnan(I_a)
    if use_segmented_sinusoidal_inertia
        fprintf('转动惯量   I_a = %.6f kg·m²  （分段正弦惯量辨识, R² = %.4f）\n', I_a, I_a_R2);
    else
        fprintf('转动惯量   I_a = %.6f kg·m²  （来自加速度扫描, R² = %.4f）\n', I_a, I_a_R2);
    end
end
fprintf('=====================================\n');

%% 3.5 自动生成飞书用结果记录（CSV 表格 + Markdown 文档）
% 电机型号 = dd 的最后一层文件夹名；结果保存在当前目录下 build 文件夹，同一电机只保留最新一行/一份
[~, motor_name] = fileparts(dd);
if isempty(motor_name), [~, motor_name] = fileparts(fileparts(dd)); end
if isempty(motor_name), motor_name = 'unknown'; end
results_dir = fullfile(cur, 'build');
if ~exist(results_dir, 'dir'), mkdir(results_dir); end
csv_path = fullfile(results_dir, 'friction_iden_results.csv');
row_date = datestr(now, 'yyyy-mm-dd HH:MM');
csv_header = '电机型号,辨识日期,K_tau_Nm/A,摩擦模型,tau_s_Nm,mu_s_Nm,v_act_rads,mu_d_Nms_rad,tau_c_pos_Nm,tau_c_neg_Nm,b_Nms_rad,tau0_Nm,I_a_kgm2,I_a_R2,R2_friction,RMSE_Nm,nUsed,备注';
% 新行内容：公式中有几个参数就填几列，其余列写 NaN
tau_s_str = num2str(tau_s, '%.4f');
if strcmpi(Result_friction.FrictionModel, 'tanh_viscous')
    % tanh 公式仅 μ_s、v_act、μ_d；τ_c、b、τ_0 不在公式中 → NaN
    mu_s_str = num2str(Result_friction.mu_s, '%.4f');
    v_act_str = num2str(Result_friction.v_act, '%.4f');
    mu_d_str = num2str(Result_friction.mu_d, '%.4f');
    tc_pos_str = 'NaN'; tc_neg_str = 'NaN'; b_str = 'NaN'; tau0_str = 'NaN';
elseif strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
    % Stribeck 公式仅 τ_s、τ_c、v_s、b、τ_0；μ_s、μ_d 不在公式中 → NaN
    mu_s_str = 'NaN'; mu_d_str = 'NaN';
    v_act_str = num2str(Result_friction.v_s, '%.4f');
    tc_pos_str = num2str(Result_friction.tau_c, '%.4f');
    tc_neg_str = num2str(-Result_friction.tau_c, '%.4f');
    b_str = num2str(Result_friction.b, '%.4f');
    tau0_str = num2str(Result_friction.tau0, '%.4f');
else
    mu_s_str = 'NaN'; v_act_str = 'NaN'; mu_d_str = 'NaN';
    tc_pos_str = num2str(Result_friction.tau_c_pos, '%.4f');
    tc_neg_str = num2str(Result_friction.tau_c_neg, '%.4f');
    b_str = num2str(Result_friction.b, '%.4f');
    tau0_str = 'NaN';
end
% 惯量列为空时写 NaN，避免 CSV/Excel 列错位
I_a_str = 'NaN'; I_a_R2_str = 'NaN';
if ~isnan(I_a)
    I_a_str = num2str(I_a, '%.6f');
    I_a_R2_str = num2str(I_a_R2, '%.4f');
end
R2_str = num2str(Result_friction.R2, '%.4f');
RMSE_str = num2str(Result_friction.RMSE, '%.4f');
n_str = num2str(Result_friction.nUsed, '%d');
remark_str = '';
new_row = sprintf('%s,%s,%.2f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n', ...
    motor_name, row_date, K_tau, Result_friction.FrictionModel, ...
    tau_s_str, mu_s_str, v_act_str, mu_d_str, tc_pos_str, tc_neg_str, b_str, tau0_str, ...
    I_a_str, I_a_R2_str, R2_str, RMSE_str, n_str, remark_str);
% CSV：同电机 + 同摩擦模型 只保留最新一行（以本次覆盖）
existing_data_rows = {};
if exist(csv_path, 'file')
    try
        txt = fileread(csv_path);
        if length(txt) > 0 && uint16(txt(1)) == 65279, txt = txt(2:end); end  % 去 BOM
        lines = strsplit(txt, '\n');
        if length(lines) < 2, lines = {}; end
        for k = 2:length(lines)
            if isempty(lines{k}), continue; end
            line_k = strtrim(lines{k});
            if isempty(line_k), continue; end
            parts_k = strsplit(line_k, ',');
            if numel(parts_k) < 4, continue; end
            row_motor = strtrim(parts_k{1});
            row_model = strtrim(parts_k{4});
            % 仅当「电机型号」或「摩擦模型」不同于当前组合时才保留旧行
            if ~(strcmp(row_motor, motor_name) && strcmp(row_model, Result_friction.FrictionModel))
                existing_data_rows{end+1} = line_k;  %#ok<AGROW>
            end
        end
    catch
        existing_data_rows = {};
    end
end
fid = fopen(csv_path, 'w');
if fid < 0
    warning('无法写入结果 CSV: %s', csv_path);
else
    fprintf(fid, '\xEF\xBB\xBF%s\n', csv_header);
    for k = 1:length(existing_data_rows)
        fprintf(fid, '%s\n', existing_data_rows{k});
    end
    fprintf(fid, '%s', new_row);
    fclose(fid);
    fprintf('  已更新表格（同型号仅保留最新）: %s\n', csv_path);
end
% 全部电机参数汇总表（Excel）：与 CSV 一致，每台电机仅保留最新一行
summary_xlsx_path = fullfile(results_dir, '全部电机参数汇总.xlsx');
try
    hdr = strsplit(csv_header, ',');
    all_rows = [existing_data_rows, {strtrim(new_row)}];
    if isempty(all_rows)
        all_rows = {};
    end
    N = length(all_rows);
    if N > 0 && length(hdr) >= 2
        ncol = length(hdr);
        C = cell(N, ncol);
        for k = 1:N
            parts = strsplit(all_rows{k}, ',');
            for j = 1:min(length(parts), ncol)
                C{k,j} = strtrim(parts{j});
            end
        end
        T_summary = cell2table(C, 'VariableNames', hdr);
        writetable(T_summary, summary_xlsx_path, 'Sheet', '全部电机参数汇总', 'WriteVariableNames', true);
        fprintf('  已更新全部电机汇总表: %s（共 %d 台电机）\n', summary_xlsx_path, N);
    end
catch e
    warning('写入全部电机汇总 Excel 失败: %s', e.message);
end
% Markdown 报告：同一电机一份，文件名不含时间，覆盖旧文件
md_name = sprintf('friction_iden_report_%s', motor_name);
md_path = fullfile(results_dir, [md_name '.md']);
fm = fopen(md_path, 'w');
if fm >= 0
    fprintf(fm, '# 摩擦与惯量辨识结果\n\n');
    fprintf(fm, '| 项目 | 值 |\n|------|----|\n');
    fprintf(fm, '| 电机型号 | %s |\n', motor_name);
    fprintf(fm, '| 辨识日期 | %s |\n', row_date);
    fprintf(fm, '| 力矩系数 K_tau | %.2f N·m/A |\n', K_tau);
    fprintf(fm, '| 摩擦模型 | %s |\n', Result_friction.FrictionModel);
    fprintf(fm, '| 静摩擦 τ_s | %.4f N·m |\n', tau_s);
    if strcmpi(Result_friction.FrictionModel, 'tanh_viscous')
        fprintf(fm, '| μ_s | %.4f N·m |\n', Result_friction.mu_s);
        fprintf(fm, '| v_act | %.4f rad/s |\n', Result_friction.v_act);
        fprintf(fm, '| μ_d | %.4f N·m·s/rad |\n', Result_friction.mu_d);
    elseif strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
        fprintf(fm, '| τ_s (Stribeck) | %.4f N·m |\n', Result_friction.tau_s);
        fprintf(fm, '| τ_c | %.4f N·m |\n', Result_friction.tau_c);
        fprintf(fm, '| v_s | %.4f rad/s |\n', Result_friction.v_s);
        fprintf(fm, '| b | %.4f N·m·s/rad |\n', Result_friction.b);
        fprintf(fm, '| τ_0 | %.4f N·m |\n', Result_friction.tau0);
    else
        fprintf(fm, '| τ_c_pos | %.4f N·m |\n', Result_friction.tau_c_pos);
        fprintf(fm, '| τ_c_neg | %.4f N·m |\n', Result_friction.tau_c_neg);
        fprintf(fm, '| b | %.4f N·m·s/rad |\n', Result_friction.b);
    end
    if ~isnan(I_a)
        fprintf(fm, '| 转动惯量 I_a | %.6f kg·m² |\n', I_a);
        fprintf(fm, '| I_a 拟合 R² | %.4f |\n', I_a_R2);
    end
    fprintf(fm, '| 摩擦拟合 R² | %.4f |\n', Result_friction.R2);
    fprintf(fm, '| 摩擦拟合 RMSE | %.4f N·m |\n', Result_friction.RMSE);
    fprintf(fm, '| 有效样本数 nUsed | %d |\n', Result_friction.nUsed);
    % 整体拟合误差（与惯量辨识统一数据/范围：加速度辨识时用加速度数据，否则用速度扫描）
    data_vel_md = read_python_upper_scan_csv(overall_data_file);
    qd_md = data_vel_md(:, 3);
    i_md = sign(qd_md) .* abs(data_vel_md(:, 2));
    tau_act_md = K_tau * i_md;
    if strcmpi(Result_friction.FrictionModel, 'tanh_viscous')
        tau_f_md = Result_friction.mu_s * tanh(qd_md / Result_friction.v_act) + Result_friction.mu_d * qd_md;
    elseif strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
        ts = Result_friction.tau_s; tc = Result_friction.tau_c; vs = Result_friction.v_s;
        b = Result_friction.b; tau0 = Result_friction.tau0;
        tau_f_md = (tc + (ts - tc)*exp(-(abs(qd_md)/vs).^2)).*sign(qd_md) + b*qd_md + tau0;
    else
        tc_pos = Result_friction.tau_c_pos; tc_neg = Result_friction.tau_c_neg; b = Result_friction.b;
        tau_f_md = (qd_md > 0) .* (tc_pos + b*qd_md) + (qd_md < 0) .* (tc_neg + b*qd_md) + (qd_md == 0) .* 0;
    end
    dt_md = diff(data_vel_md(:, 1));
    if length(dt_md) > 0
        qdd_md = diff(qd_md) ./ dt_md; qdd_md = [qdd_md(1); qdd_md];
    else
        qdd_md = zeros(size(qd_md));
    end
    if ~isnan(I_a)
        tau_model_md = I_a * qdd_md + tau_f_md;
    else
        tau_model_md = tau_f_md;
    end
    err_full_md = tau_act_md - tau_model_md;
    idx_md_ok = true(size(tau_act_md));
    if isfinite(accel_exclude_above)
        idx_md_ok = idx_md_ok & (abs(qdd_md) <= accel_exclude_above);
    end
    if isfinite(tau_exclude_above)
        idx_md_ok = idx_md_ok & (abs(tau_act_md) <= tau_exclude_above);
    end
    err_full_md = err_full_md(idx_md_ok);
    tau_act_md = tau_act_md(idx_md_ok);
    RMSE_overall = sqrt(mean(err_full_md.^2));
    ss_res = sum(err_full_md.^2);
    ss_tot = sum((tau_act_md - mean(tau_act_md)).^2);
    R2_overall = 1 - ss_res / max(ss_tot, 1e-12);
    fprintf(fm, '| 整体拟合 RMSE | %.4f N·m |\n', RMSE_overall);
    fprintf(fm, '| 整体拟合 R² | %.4f |\n', R2_overall);
    fprintf(fm, '\n---\n*由 run_full_iden_three_files 自动生成，可复制上表到飞书文档。*\n');
    fclose(fm);
    fprintf('  已生成文档: %s（可复制内容到飞书文档）\n', md_path);
end

%% 4. 拟合模型力矩 vs 实际力矩对比图（两个子图 = 不含惯量 / 含惯量，数据与整体拟合误差统一）
fprintf('\n===== 拟合模型力矩 vs 实际力矩对比 =====\n');
data_vel = read_python_upper_scan_csv(overall_data_file);  % 与整体拟合误差同一数据源
t_vel = data_vel(:, 1);
i_act_full = data_vel(:, 2);
qd_vel = data_vel(:, 3);  % 已是 rad/s
i_act_full = sign(qd_vel) .* abs(i_act_full);  % 与辨识时一致
% 实际力矩（由电流×K_tau）
tau_act = K_tau * i_act_full;
% 拟合模型力矩：仅摩擦项 τ_f
if strcmpi(Result_friction.FrictionModel, 'tanh_viscous')
    mu_s = Result_friction.mu_s; v_act = Result_friction.v_act; mu_d = Result_friction.mu_d;
    tau_f_model = mu_s * tanh(qd_vel / v_act) + mu_d * qd_vel;
elseif strcmpi(Result_friction.FrictionModel, 'stribeck_viscous')
    ts = Result_friction.tau_s; tc = Result_friction.tau_c; vs = Result_friction.v_s;
    b = Result_friction.b; tau0 = Result_friction.tau0;
    tau_f_model = (tc + (ts - tc)*exp(-(abs(qd_vel)/vs).^2)).*sign(qd_vel) + b*qd_vel + tau0;
else
    tc_pos = Result_friction.tau_c_pos; tc_neg = Result_friction.tau_c_neg; b = Result_friction.b;
    tau_f_model = (qd_vel > 0) .* (tc_pos + b*qd_vel) + (qd_vel < 0) .* (tc_neg + b*qd_vel) + (qd_vel == 0) .* 0;
end
% 角加速度 q̈（由速度数值微分，用于含惯量模型）
dt_vel = diff(t_vel);
if length(dt_vel) > 0
    qdd_vel = diff(qd_vel) ./ dt_vel;
    qdd_vel = [qdd_vel(1); qdd_vel];  % 与 t_vel 同长
else
    qdd_vel = zeros(size(qd_vel));
end
% 拟合模型力矩（含惯量）：I_a×q̈ + τ_f；无 I_a 时仅用 τ_f
if ~isnan(I_a)
    tau_model_with_inertia = I_a * qdd_vel + tau_f_model;
else
    tau_model_with_inertia = tau_f_model;  % 未辨识 I_a，仅摩擦
end
% 平均后力矩：滑动窗口平均（窗长 101 点）
win = 101;
if length(tau_act) >= win
    tau_avg = movmean(tau_act, win);
else
    tau_avg = tau_act;
end
% 横坐标为时间 (行数/Fs)，绘制拟合后整体模型误差（实际力矩 - 拟合模型力矩）
idx_row = (1:length(tau_act))' / Fs;
err_friction_only = tau_act - tau_f_model;       % 仅摩擦模型误差
err_with_inertia   = tau_act - tau_model_with_inertia;  % 含惯量模型误差
% 剔除特别大的角加速度与力矩：整体误差统计与整体误差图均不用 |q̈|>accel_exclude_above 或 |τ|>tau_exclude_above 的点
idx_accel_ok = true(size(qdd_vel));
if isfinite(accel_exclude_above)
    idx_accel_ok = abs(qdd_vel) <= accel_exclude_above;
    n_excl = sum(~idx_accel_ok);
    if n_excl > 0
        fprintf('  剔除 |q̈|>%.4g 的点：%d / %d\n', accel_exclude_above, n_excl, length(qdd_vel));
    end
end
idx_tau_ok = true(size(tau_act));
if isfinite(tau_exclude_above)
    idx_tau_ok = abs(tau_act) <= tau_exclude_above;
    n_excl_tau = sum(~idx_tau_ok);
    if n_excl_tau > 0
        fprintf('  剔除 |τ|>%.4g 的点：%d / %d\n', tau_exclude_above, n_excl_tau, length(tau_act));
    end
end
idx_ok = idx_accel_ok & idx_tau_ok;
err_ok = err_with_inertia(idx_ok);
tau_act_ok = tau_act(idx_ok);
RMSE_overall_sec4 = sqrt(mean(err_ok.^2));
ss_res_sec4 = sum(err_ok.^2);
ss_tot_sec4 = sum((tau_act_ok - mean(tau_act_ok)).^2);
R2_overall_sec4 = 1 - ss_res_sec4 / max(ss_tot_sec4, 1e-12);
figure('Name', '拟合后整体模型误差');
subplot(2, 1, 1);
plot(idx_row, err_friction_only, 'b-', 'LineWidth', 0.6);
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('误差 (N\cdot m)');
title('整体模型误差（仅摩擦）：\\tau_{act} - \\tau_{friction}');
subplot(2, 1, 2);
plot(idx_row, err_with_inertia, 'r-', 'LineWidth', 0.6);
grid on; xlabel(sprintf('时间 (s), %d Hz', Fs)); ylabel('误差 (N\cdot m)');
if ~isnan(I_a)
    title('整体模型误差（含惯量）：\\tau_{act} - (I_a\\ddot{q}+\\tau_f)');
else
    title('整体模型误差（未辨识 I_a，同仅摩擦）');
end
if strcmp(overall_data_file, file_accel)
    fprintf('  整体拟合误差（加速度数据，与惯量辨识统一）: RMSE = %.4f N·m, R² = %.4f\n', RMSE_overall_sec4, R2_overall_sec4);
else
    fprintf('  整体拟合误差: RMSE = %.4f N·m, R² = %.4f\n', RMSE_overall_sec4, R2_overall_sec4);
end
fprintf('  已生成图：横坐标行数，纵坐标拟合后整体模型误差（仅摩擦 / 含惯量）。\n');

% 整体误差图：横坐标为速度/加速度，纵坐标为误差（已剔除 |q̈|、|τ| 过大的点）
qd_ok = qd_vel(idx_ok);
qdd_ok = qdd_vel(idx_ok);
err_plot = err_with_inertia(idx_ok);
figure('Name', '整体误差图');
subplot(2, 1, 1);
plot(qd_ok, err_plot, 'b.', 'MarkerSize', 3);
grid on; xlabel('角速度 \\dot{q} (rad/s)'); ylabel('误差 (N\\cdot m)');
title('整体模型误差 vs 速度：\\tau_{act} - (I_a\\ddot{q}+\\tau_f)');
subplot(2, 1, 2);
plot(qdd_ok, err_plot, 'r.', 'MarkerSize', 3);
grid on; xlabel('角加速度 \\ddot{q} (rad/s^2)'); ylabel('误差 (N\\cdot m)');
if isfinite(accel_exclude_above)
    title(sprintf('整体模型误差 vs 加速度（已剔除 |q̈|>%.4g）', accel_exclude_above));
else
    title('整体模型误差 vs 加速度');
end
fprintf('  已生成图：整体误差图（横轴速度/加速度，纵轴误差，已剔除大角加速度点）。\n');

% 先保存所有附图到 build，再导出 Excel（Excel 中写附图路径）
allfigs = findobj(0, 'Type', 'figure');
saveNames = { '静摩擦辨识数据', '摩擦辨识（tanh 模型）', '摩擦辨识（Stribeck 模型）', 'Joint friction identification', '截取后数据', '惯量辨识', '加速度拟合图', '整体拟合图', '拟合后整体模型误差', '整体误差图' };
saveFiles = { '静摩擦辨识图', '速度拟合图', '速度拟合图', '速度拟合图', '截取后数据', '惯量辨识', '加速度拟合图', '整体拟合图', '整体拟合误差图', '整体误差图' };
for i = 1:length(allfigs)
    n = get(allfigs(i), 'Name');
    for j = 1:length(saveNames)
        if strcmp(n, saveNames{j})
            png_path = fullfile(results_dir, [saveFiles{j}, '_', motor_name, '.png']);
            saveas(allfigs(i), png_path);
            fprintf('  已保存: %s\n', png_path);
            break;
        end
    end
end

% 生成「静摩擦 / 动摩擦 / 转动惯量」Excel（每表含数据 + 附图路径）
if exist('export_motor_spec_to_excel', 'file')
    export_motor_spec_to_excel('MotorName', motor_name, 'FigureDir', results_dir, ...
        'K_tau', K_tau, 'tau_s', tau_s, 'file_static', file_static, ...
        'Result_friction', Result_friction, 'I_a', I_a, 'I_a_R2', I_a_R2, ...
        'RMSE_overall', RMSE_overall_sec4, 'R2_overall', R2_overall_sec4, ...
        'OutPath', fullfile(results_dir, [motor_name, '_', Result_friction.FrictionModel, '_静摩擦_动摩擦_转动惯量.xlsx']));
else
    fprintf('  未找到 export_motor_spec_to_excel，跳过 Excel 导出。\n');
end

%% 局部函数：由 [t, current, qd] 得到 qdd（数值微分，不平滑）
function qdd = accel_from_vel(d)
if size(d,1) < 2
    qdd = zeros(size(d,1), 1);
    return;
end
t = d(:,1); qd = d(:,3);
dt = diff(t);
qdd = diff(qd) ./ dt;
qdd = [qdd(1); qdd];
end
