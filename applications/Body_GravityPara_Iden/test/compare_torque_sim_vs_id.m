% compare_torque_sim_vs_id  实机/仿真力矩与逆动力学模型力矩对比
%
% 从测试诊断文件 dyn_para_test_sim.csv 读取实机（或仿真）关节力矩，
% 与 URDF 逆动力学模型计算力矩进行对比。
%
% 逆动力学计算方式（机器人悬空、固定基座）：
%   按 chain 单独算：左腿 chain 仅左腿 q/qd/qdd 来自数据，右腿与手臂固定为 0；
%   右腿 chain 仅右腿 q/qd/qdd 来自数据，左腿与手臂固定为 0。参考 sim_forward_dynamics / compare_with_isaac_lab 的“单 chain”思路。
%
% 两种分析方式：
%   1. 不进行往返轨迹对齐：按时间序列逐点对比 tau_real(t) vs tau_model(t)
%   2. 双文件往返对齐：输入正向轨迹 CSV + 逆向轨迹 CSV。验证第二文件逆序后关节角与第一文件相等、速度相反，
%      再取 (tau_正+tau_反)/2 得到惯性+重力力矩，与模型 ID(q,qd_avg,qdd_avg)=M*qdd+C*qd+G 对比。
%
% 用法（等价，任选其一）：
%   compare_torque_sim_vs_id('path/to/file.csv')
%   compare_torque_sim_vs_id('csv_file', 'path/to/file.csv')
%   compare_torque_sim_vs_id('path/to/fwd.csv', 'align', true, 'csv_file_reverse', 'path/to/rev.csv')
%   csv_file 为必选（正向轨迹）；align=true 时 csv_file_reverse 为必选（逆向轨迹）。
%
% 输出（三个参数均返回）：
%   rmse_no_align - 12×1，不对齐时各关节 RMSE；若 fit_tau_err_qdd=true 则为拟合残差 RMSE (N·m)
%   rmse_align    - 12×1 或 []，对齐时各关节 RMSE（未做对齐则为 []）
%   data          - struct：time, tau_real, tau_model, tau_model_plus_friction（动力学+膝摩擦），q, qd, qdd
%
% 可选：在力矩对比中增加第三条线「动力学+摩擦力矩」（仅膝关节 L4/R4 加摩擦，参数可覆盖）：
%   'add_friction_line', true  才计算并绘制该线，默认 false（先确认 MATLAB 与 Gazebo 一致再加摩擦）
%   'knee_friction_tau_s', 3.12, 'knee_friction_tau_c', 2.0267, 'knee_friction_b', 0.4441
% 可选：逆动力学用到的 qd/qdd 来源。推荐「先滤 q 或 qd 再差分得 qdd」，不直接滤 qdd（避免相位滞后、惯性项判断失真）。
%   'qd_qdd_from_q', false（默认）：qd=表格，qdd=中心差分(qd)，不滤波。
%   'qd_qdd_from_q', true：q 低通 fc_q_lpf → 中心差分 qd → 中心差分 qdd；fc_qdd_lpf>0 时再对 qdd 低通，fc_qdd_lpf<=0 则不滤 qdd（推荐）。
%   'qd_smooth_then_qdd', true：仅对表格 qd 低通 fc_q_lpf → 中心差分得 qdd（不滤 q、不滤 qdd），最小可执行方案。
% 可选：差分 qdd vs FD qdd 对比（先不滤波）：'compare_qdd_diff_vs_fd', true → 额外画图并输出 RMSE(qdd_diff - qdd_fd)
% 可选：τ_err = a·qdd + b 逐关节线性回归（惯性项量化）：'fit_tau_err_qdd', true → 输出斜率 a、R²、RMSE(原始/拟合残差)
% 可选：将所有生成的图保存为 PNG：'save_png', true；默认保存到当前目录下的 fig_png 文件夹，也可用 'save_png_dir' 指定路径
% 可选：时间对齐（MATLAB 比 Gazebo 早一刻时）：'time_align_shift', 1 → 比较 real(2:n) 与 model(1:n-1)
% 可选：踝关节 L5/L6/R5/R6 轴向与数据相反时：'flip_ankle_sign', true → 模型力矩取反再对比（仅当确认为符号/轴向差异时用）
%
% 踝关节（每腿后两关节 L5/L6, R5/R6）说明：
%   实机 E1HW 为并联踝（2 电机→2 关节），parallel_ankle_solve_state/cmd 做电机↔关节转换，等效动力学与“两根独立回转关节”不同。
%   Gazebo/URDF 与 MATLAB 本脚本均为两根独立回转关节（标准开链 M·q̈+C+g=τ）。故 Gazebo–MATLAB 对比时踝关节可用同一套模型；
%   实机–仿真踝差异大是预期（实机并联踝 vs 仿真简单链）。不应对踝做取反“修正”，除非确认为轴向/符号约定不同。
%
% 依赖：robot_algorithm/Dynamics/E1/compute_e1_limb_dynamics.m（E1 按肢体逆动力学，每条腿 6 维）

function varargout = compare_torque_sim_vs_id(varargin)
% 兼容“第一参数直接为文件路径”的写法（原 run_real_test_compare_torque 用法）
param_names = {'csv_file', 'csv_file_reverse', 'n_bins', 'qdd_smooth', 'max_points', 'active_joint', 'align', 'qdd_source', ...
    'qd_qdd_from_q', 'qd_smooth_then_qdd', 'fc_q_lpf', 'fc_qdd_lpf', 'compare_qdd_diff_vs_fd', 'fit_tau_err_qdd', 'save_png', 'save_png_dir', ...
    'knee_friction_tau_s', 'knee_friction_tau_c', 'knee_friction_b', 'add_friction_line', 'time_align_shift', 'flip_ankle_sign'};
if nargin >= 1 && ischar(varargin{1}) && ~any(strcmp(varargin{1}, param_names))
    varargin = [{'csv_file', varargin{1}}, varargin(2:end)];
end
p = inputParser;
addParameter(p, 'csv_file', '', @ischar);
addParameter(p, 'csv_file_reverse', '', @ischar);  % 对齐时必选：逆向轨迹 CSV
addParameter(p, 'n_bins', 50, @isnumeric);   % 对齐分析时的位置分箱数
addParameter(p, 'qdd_smooth', 0, @isnumeric); % qdd 数值微分时的平滑点数（0 表示不平滑，默认不滤波）
addParameter(p, 'max_points', Inf, @isnumeric);   % 逆动力学逐点计算时最多使用的点数，Inf=不降采样
addParameter(p, 'active_joint', 0, @isnumeric);  % 单关节往返时的主动关节 1-12，0=自动检测（范围最大的关节）
addParameter(p, 'align', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));  % 是否进行单关节往返对齐分析，默认不对齐
addParameter(p, 'qdd_source', 'diff', @(x) ischar(x) && ismember(lower(x), {'diff','csv','fd'})); % qdd 来源：默认 diff=按表格速度差分计算
addParameter(p, 'qd_qdd_from_q', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % false=qd 用表格、qdd=中心差分(qd)；true=q 低通→中心差分 qd→qdd，可选 qdd 低通
addParameter(p, 'qd_smooth_then_qdd', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=仅对表格 qd 低通→中心差分得 qdd（不直接滤 qdd）
addParameter(p, 'fc_q_lpf', 50, @isnumeric);   % q 或 qd 低通截止 (Hz)，qd_qdd_from_q 或 qd_smooth_then_qdd 时有效
addParameter(p, 'fc_qdd_lpf', 0, @isnumeric);  % qdd 低通 (Hz)，仅 qd_qdd_from_q=true 且 >0 时对 qdd 低通；<=0 则不滤 qdd（推荐）
addParameter(p, 'compare_qdd_diff_vs_fd', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=额外绘制并输出「差分 qdd vs FD qdd」对比（均不滤波）
addParameter(p, 'fit_tau_err_qdd', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=做 τ_err = a·qdd + b 逐关节线性回归（惯性项量化）
addParameter(p, 'save_png', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=将所有生成的图保存为 PNG
addParameter(p, 'save_png_dir', '', @ischar); % PNG 保存目录，空则用当前目录
addParameter(p, 'knee_friction_tau_s', 3.12, @isnumeric);
addParameter(p, 'knee_friction_tau_c', 2.0267, @isnumeric);
addParameter(p, 'knee_friction_b', 0.4441, @isnumeric);
addParameter(p, 'add_friction_line', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true 时才画「动力学+摩擦力矩」线
addParameter(p, 'time_align_shift', 0, @isnumeric); % 1=MATLAB 比 Gazebo 早一刻，用 real(1:n-1) 对 model(2:n) 对齐
addParameter(p, 'flip_ankle_sign', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true 时踝关节 L5/L6/R5/R6 的模型力矩取反再对比（试 Gazebo 轴向相反）
parse(p, varargin{:});
opts = p.Results;
% 保证 principle 与路径（直接运行本脚本时需先加路径）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end
opts.align = logical(opts.align);
opts.qdd_source = lower(opts.qdd_source);
opts.qd_qdd_from_q = logical(opts.qd_qdd_from_q);
opts.qd_smooth_then_qdd = logical(opts.qd_smooth_then_qdd);
opts.compare_qdd_diff_vs_fd = logical(opts.compare_qdd_diff_vs_fd);
opts.fit_tau_err_qdd = logical(opts.fit_tau_err_qdd);
opts.save_png = logical(opts.save_png);
opts.add_friction_line = logical(opts.add_friction_line);
opts.flip_ankle_sign  = logical(opts.flip_ankle_sign);
if opts.save_png && isempty(opts.save_png_dir)
    opts.save_png_dir = fullfile(pwd, 'fig_png');
end
if opts.align && isempty(opts.csv_file_reverse)
    error('对齐模式需指定两个文件：正向轨迹 csv_file 与逆向轨迹 csv_file_reverse。');
end

%% 1. 读取数据文件，csv_file 为必选参数
if isempty(opts.csv_file)
    error('请指定数据文件，例如: compare_torque_sim_vs_id(''csv_file'', ''path/to/file.csv'')');
end
if ~exist(opts.csv_file, 'file')
    error('未找到数据文件: %s', opts.csv_file);
end

fprintf('===== 实机/仿真力矩 vs 逆动力学模型力矩 对比 =====\n');
fprintf('数据文件: %s\n', opts.csv_file);

% 使用 read_leg_joint_csv 读取，得到 time, cmd_leg_l, pos_leg_l, vel_leg_l, torque_leg_l (各 n×6) 及右腿
ensure_body_gravity_para_iden_path();
data = read_leg_joint_csv(opts.csv_file);

time     = data.time;
% 内部仍用 n×12：左腿 1:6，右腿 7:12
q        = [data.pos_leg_l,   data.pos_leg_r];    % n×12
qd       = [data.vel_leg_l,   data.vel_leg_r];
tau_real = [data.torque_leg_l, data.torque_leg_r];
% 如需 cmd：cmd_leg_l = data.cmd_leg_l (n×6), cmd_leg_r = data.cmd_leg_r (n×6)

joint_suffix = {'leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_l6_joint', ...
    'leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint','leg_r6_joint'};

% 间隔时间统一按第一列 time 计算（见下方 dt_vec）
dt_default = median(diff(time));
if dt_default <= 0 || isnan(dt_default), dt_default = 0.002; end

%% 1.5 可选降采样（仅当 max_points 为有限值且数据点数超出时）
N_orig = length(time);
if isfinite(opts.max_points) && N_orig > opts.max_points
    idx = round(linspace(1, N_orig, opts.max_points));
    idx = unique(idx);
    time = time(idx);
    q = q(idx, :);
    qd = qd(idx, :);
    tau_real = tau_real(idx, :);
    if isfield(data, 'acc_leg_l')
        data.acc_leg_l = data.acc_leg_l(idx, :);
        data.acc_leg_r = data.acc_leg_r(idx, :);
    end
    dt_default = median(diff(time));
    if dt_default <= 0 || isnan(dt_default), dt_default = 0.002; end
    fprintf('数据点数 %d > max_points=%d，已降采样至 %d 点。\n', N_orig, opts.max_points, length(time));
end

%% 1.6 可选：差分 qdd vs FD qdd 对比（均不滤波）
% 差分：表格 qd 做中心差分得 qdd_diff；FD：按单链 FD 得 qdd_fd。
if opts.compare_qdd_diff_vs_fd
    n = length(time);
    qdd_diff = zeros(n, 12);
    for j = 1:12
        qdd_diff(:, j) = central_diff(qd(:, j), time);
    end
    qdd_fd = zeros(n, 12);
    fprintf('正在按单链计算 FD（左/右腿各 6 维）得到 qdd_fd（与差分 qdd 对比，均不滤波）…\n');
    for k = 1:n
        q_left = q(k, 1:6)';   qd_left = qd(k, 1:6)';   tau_left = tau_real(k, 1:6)';
        q_right = q(k, 7:12)'; qd_right = qd(k, 7:12)'; tau_right = tau_real(k, 7:12)';
        q_left(~isfinite(q_left)) = 0;   qd_left(~isfinite(qd_left)) = 0;   tau_left(~isfinite(tau_left)) = 0;
        q_right(~isfinite(q_right)) = 0; qd_right(~isfinite(qd_right)) = 0; tau_right(~isfinite(tau_right)) = 0;
        qdd_fd(k, 1:6)  = compute_e1_limb_forward_dynamics('left_leg',  q_left,  qd_left,  tau_left)';
        qdd_fd(k, 7:12) = compute_e1_limb_forward_dynamics('right_leg', q_right, qd_right, tau_right)';
    end
    err_qdd = qdd_diff - qdd_fd;
    rmse_qdd = sqrt(mean(err_qdd.^2, 1));
    fprintf('\n--- 差分 qdd vs FD qdd（均不滤波） ---\n');
    fprintf('各关节 RMSE(qdd_diff - qdd_fd): '); fprintf(' %.4f', rmse_qdd); fprintf(' rad/s^2\n');
    fprintf('全关节平均 RMSE: %.4f rad/s^2\n', mean(rmse_qdd));
    fig_qdd = figure('Name', '逆动力学_角加速度对比_差分vsFD_不滤波');
    for j = 1:12
        ax = subplot(3, 4, j);
        plot(ax, time, qdd_diff(:, j), 'b-', 'DisplayName', 'qdd\_diff');
        hold(ax, 'on');
        plot(ax, time, qdd_fd(:, j), 'r--', 'DisplayName', 'qdd\_FD');
        xlabel(ax, '时间 (s)'); ylabel(ax, 'rad/s^2');
        title(ax, sprintf('%s RMSE=%.3f', joint_suffix{j}, rmse_qdd(j)));
        legend(ax, 'Location', 'best'); grid(ax, 'on');
    end
    sgtitle(fig_qdd, '角加速度对比：差分(表格 qd 中心差分) vs FD(q,qd,\tau)，不滤波');
    save_fig_png(opts, 'qdd_compare');
    figure('Name', '逆动力学_角加速度误差_差分减FD_不滤波');
    for j = 1:12
        subplot(3, 4, j);
        plot(time, err_qdd(:, j), 'k-');
        xlabel('时间 (s)'); ylabel('rad/s^2');
        title(sprintf('%s', joint_suffix{j}));
        grid on;
    end
    sgtitle('qdd 误差：qdd\_diff - qdd\_FD（不滤波）');
    save_fig_png(opts, 'qdd_error');
end

%% 2. qdd：三种来源（默认：按表格速度差分计算）
%   - 'diff': 按表格速度差分计算 qdd，qdd(k) ≈ (qd(k+1)-qd(k))/dt（默认）
%   - 'csv' : 直接使用 CSV 中的角加速度列（acc_leg_* 或 qdd_*_data）
%   - 'fd'  : 按单链 FD（左/右腿各 6 维）计算 qdd，再做 ID，与逆动力学一致
% 当 qdd_source='csv' 且表格无加速度列时，自动退回 'diff'。

use_table_acc = isfield(data, 'acc_leg_l') && isfield(data, 'acc_leg_r');
if strcmp(opts.qdd_source, 'fd')
    % 按单链 FD：左腿 FD(q_left,qd_left,tau_left)、右腿 FD(q_right,qd_right,tau_right)，与逆动力学一致
    n = length(time);
    qdd = zeros(n, 12);
    fprintf('qdd_source = ''fd'': 正在按单链 FD（左/右腿各 6 维）计算每一拍的加速度用于逆动力学…\n');
    for k = 1:n
        q_left = q(k, 1:6)';   qd_left = qd(k, 1:6)';   tau_left = tau_real(k, 1:6)';
        q_right = q(k, 7:12)'; qd_right = qd(k, 7:12)'; tau_right = tau_real(k, 7:12)';
        q_left(~isfinite(q_left)) = 0;   qd_left(~isfinite(qd_left)) = 0;   tau_left(~isfinite(tau_left)) = 0;
        q_right(~isfinite(q_right)) = 0; qd_right(~isfinite(qd_right)) = 0; tau_right(~isfinite(tau_right)) = 0;
        qdd(k, 1:6)  = compute_e1_limb_forward_dynamics('left_leg',  q_left,  qd_left,  tau_left)';
        qdd(k, 7:12) = compute_e1_limb_forward_dynamics('right_leg', q_right, qd_right, tau_right)';
    end
    if opts.qdd_smooth > 0
        w = ones(opts.qdd_smooth, 1) / opts.qdd_smooth;
        for j = 1:12
            qdd(:, j) = conv(qdd(:, j), w, 'same');
        end
    end
elseif strcmp(opts.qdd_source, 'csv') && use_table_acc
    qdd = [data.acc_leg_l, data.acc_leg_r];
    fprintf('已采用表格中的角加速度 (acc_leg_* / qdd_*_data) 作为 qdd。\n');
else
    % diff：qd/qdd 来源二选一
    % 1) qd_qdd_from_q=true（推荐）：在「干净的 q」上做微分，避免电机 qd 噪声被放大
    %    q 低通 fc_q_lpf → 中心差分得 qd → 中心差分得 qdd → qdd 再低通 fc_qdd_lpf
    % 2) qd_qdd_from_q=false：用表格 qd，qdd = diff(qd)/dt，可选 qdd_smooth 滑动平均
    if opts.qd_qdd_from_q
        if strcmp(opts.qdd_source, 'csv') && ~use_table_acc
            fprintf('表格中无角加速度列，从 q 推导 qd/qdd（q 低通→中心差分）。\n');
        else
            if opts.fc_qdd_lpf > 0
                fprintf('已从 q 推导 qd/qdd：q 低通 %.0f Hz → 中心差分 qd → 中心差分 qdd → qdd 低通 %.0f Hz。\n', opts.fc_q_lpf, opts.fc_qdd_lpf);
            else
                fprintf('已从 q 推导 qd/qdd：q 低通 %.0f Hz → 中心差分 qd → 中心差分 qdd（qdd 不低通，推荐）。\n', opts.fc_q_lpf);
            end
        end
        fs = 1 / dt_default;
        for j = 1:12
            q_f = lowpass_butter2_zerophase(q(:, j), opts.fc_q_lpf, fs);
            q(:, j) = q_f;
            qd(:, j) = central_diff(q_f, time);
        end
        for j = 1:12
            qdd_raw = central_diff(qd(:, j), time);
            if opts.fc_qdd_lpf > 0
                qdd(:, j) = lowpass_butter2_zerophase(qdd_raw, opts.fc_qdd_lpf, fs);
            else
                qdd(:, j) = qdd_raw;
            end
        end
    elseif opts.qd_smooth_then_qdd
        % 最小可执行方案：仅对表格 qd 低通 → 中心差分得 qdd，不直接滤 qdd
        if strcmp(opts.qdd_source, 'csv') && ~use_table_acc
            fprintf('表格中无角加速度列，qd 低通 %.0f Hz → 中心差分得 qdd（不滤 qdd）。\n', opts.fc_q_lpf);
        else
            fprintf('逆动力学：qd 低通 %.0f Hz → 中心差分得 qdd（不直接滤 qdd）。\n', opts.fc_q_lpf);
        end
        fs = 1 / dt_default;
        for j = 1:12
            qd_f = lowpass_butter2_zerophase(qd(:, j), opts.fc_q_lpf, fs);
            qd(:, j) = qd_f;
            qdd(:, j) = central_diff(qd_f, time);
        end
    else
        % qd 用表格实测，qdd 用 qd 中心差分，不滤波
        if strcmp(opts.qdd_source, 'csv') && ~use_table_acc
            fprintf('表格中无角加速度列，qd 用表格、qdd 用 qd 中心差分（不滤波）。\n');
        else
            fprintf('逆动力学：qd 用表格实测，qdd 用 qd 中心差分（不滤波）。\n');
        end
        for j = 1:12
            qdd(:, j) = central_diff(qd(:, j), time);
        end
        if opts.qdd_smooth > 0
            w = ones(opts.qdd_smooth, 1) / opts.qdd_smooth;
            for j = 1:12
                qdd(:, j) = conv(qdd(:, j), w, 'same');
            end
        end
    end
end
% 图名后缀：根据 qdd 来源、是否滤波、是否拟合 qdd 生成，便于区分不同运行
if strcmp(opts.qdd_source, 'fd')
    fig_run_tag = 'qdd来自FD';
elseif strcmp(opts.qdd_source, 'csv')
    fig_run_tag = 'qdd来自CSV';
else
    if opts.qd_qdd_from_q || opts.qd_smooth_then_qdd
        fig_run_tag = '滤波';
    else
        fig_run_tag = '不滤波';
    end
end
if opts.fit_tau_err_qdd
    fig_run_tag = [fig_run_tag, '_拟合qdd'];
else
    fig_run_tag = [fig_run_tag, '_不拟合qdd'];
end
% 逆动力学要求 q,qd,qdd 非 NaN/Inf，将非有限值置 0 后再调用
n_bad_qdd = sum(~isfinite(qdd(:)));
n_bad_q = sum(~isfinite(q(:)));
n_bad_qd = sum(~isfinite(qd(:)));
qdd(~isfinite(qdd)) = 0;
q(~isfinite(q)) = 0;
qd(~isfinite(qd)) = 0;
if n_bad_qdd > 0 || n_bad_q > 0 || n_bad_qd > 0
    fprintf('警告：数据中含非有限值 (q:%d, qd:%d, qdd:%d)，已置 0 后参与逆动力学。\n', n_bad_q, n_bad_qd, n_bad_qdd);
end

%% 3. 逆动力学计算（按肢体分开算，每条腿 6 维 q/qd/qdd，M/C/G 为 6×6/6×1）
tau_model = zeros(length(time), 12);
fprintf('正在逐点计算逆动力学 (共 %d 点，左/右腿各 1 次/点)...\n', length(time));
for k = 1:length(time)
    q_left   = q(k, 1:6)';
    qd_left  = qd(k, 1:6)';
    qdd_left = qdd(k, 1:6)';
    tau_L = compute_e1_limb_dynamics('left_leg', q_left, qd_left, qdd_left);
    tau_model(k, 1:6) = tau_L';

    q_right   = q(k, 7:12)';
    qd_right  = qd(k, 7:12)';
    qdd_right = qdd(k, 7:12)';
    tau_R = compute_e1_limb_dynamics('right_leg', q_right, qd_right, qdd_right);
    tau_model(k, 7:12) = tau_R';
end

% 可选：动力学+摩擦力矩（仅当 add_friction_line=true 时计算并参与对比/绘图）
if opts.add_friction_line
    tau_model_plus_friction = tau_model;
    tau_s = opts.knee_friction_tau_s;
    tau_c = opts.knee_friction_tau_c;
    b_vis = opts.knee_friction_b;
    for k = 1:length(time)
        for j = [4, 10]
            tau_f = knee_friction_torque(qd(k,j), tau_real(k,j), tau_s, tau_c, b_vis);
            tau_model_plus_friction(k, j) = tau_model(k, j) + tau_f;
        end
    end
    fprintf('已计算「动力学+摩擦力矩」（仅膝 L4/R4），并参与对比与绘图。\n');
else
    tau_model_plus_friction = [];
end
fprintf('数据点数: %d, 时长: %.3f s\n', length(time), time(end));

% 可选：踝关节 L5/L6/R5/R6 模型力矩取反（试 Gazebo 轴向与 MATLAB 相反）
if opts.flip_ankle_sign
    tau_model_plot = tau_model;
    tau_model_plot(:, [5,6,11,12]) = -tau_model_plot(:, [5,6,11,12]);
    if opts.add_friction_line
        tau_model_plus_friction(:, [5,6,11,12]) = -tau_model_plus_friction(:, [5,6,11,12]);
    end
    fprintf('已对踝关节 L5/L6/R5/R6 的模型力矩取反后再对比。\n');
else
    tau_model_plot = tau_model;
end

% 可选：MATLAB 比 Gazebo 早一刻时，用 real(2:n) 对 model(1:n-1) 对齐
if opts.time_align_shift == 1
    n = length(time);
    time_a     = time(2:n);
    tau_real_a = tau_real(2:n, :);
    tau_model_a = tau_model_plot(1:n-1, :);
    q_a = q(2:n, :); qd_a = qd(2:n, :); qdd_a = qdd(2:n, :);
    if opts.add_friction_line
        tau_fric_a = tau_model_plus_friction(1:n-1, :);
    end
    fprintf('已做时间对齐：MATLAB 早一刻，比较 real(2:n) 与 model(1:n-1)。\n');
else
    time_a     = time;
    tau_real_a = tau_real;
    tau_model_a = tau_model_plot;
    q_a = q; qd_a = qd; qdd_a = qdd;
    if opts.add_friction_line
        tau_fric_a = tau_model_plus_friction;
    end
end
n_comp = size(tau_real_a, 1);

%% 4. 分析一：不进行往返轨迹对齐（按时间逐点对比，可选时间对齐）
err_no_align = tau_real_a - tau_model_a;
rmse_no_align = sqrt(mean(err_no_align.^2, 1));
mae_no_align = mean(abs(err_no_align), 1);
max_abs_no_align = max(abs(err_no_align), [], 1);

% 先滤波再拟合：若启用拟合，则图与日志均用「减去 a·qdd+b 后的残差」
if opts.fit_tau_err_qdd
    fprintf('\n--- τ_err = a·qdd + b 逐关节线性回归（以下图与日志均用拟合残差） ---\n');
    slope_a = zeros(1, 12);
    intercept_b = zeros(1, 12);
    R2 = zeros(1, 12);
    err_display = zeros(n_comp, 12);
    for j = 1:12
        y = err_no_align(:, j);
        x = qdd_a(:, j);
        X = [x(:), ones(length(x), 1)];
        beta = X \ y(:);
        slope_a(j) = beta(1);
        intercept_b(j) = beta(2);
        y_fit = X * beta;
        err_display(:, j) = y(:) - y_fit;
        ss_tot = sum((y(:) - mean(y)).^2);
        ss_res = sum(err_display(:, j).^2);
        if ss_tot > 0
            R2(j) = 1 - ss_res / ss_tot;
        else
            R2(j) = 0;
        end
    end
    rmse_display = sqrt(mean(err_display.^2, 1));
    mae_display = mean(abs(err_display), 1);
    max_display = max(abs(err_display), [], 1);
    fprintf('关节              斜率 a (N·m/(rad/s²))  截距 b (N·m)   R²      RMSE(原始)  RMSE(拟合残差)\n');
    for j = 1:12
        fprintf('%-16s %12.4f   %10.4f   %6.3f   %10.4f   %10.4f\n', ...
            joint_suffix{j}, slope_a(j), intercept_b(j), R2(j), rmse_no_align(j), rmse_display(j));
    end
    fprintf('说明：斜率 a 为负表示模型惯量偏大；以下打印与图中误差均为「拟合残差」。\n');
    j_l3 = 3;
    fprintf('\n[L3] 斜率 a = %.4f N·m/(rad/s²)，R² = %.3f，RMSE 原始 = %.4f → 拟合残差 = %.4f N·m\n', ...
        slope_a(j_l3), R2(j_l3), rmse_no_align(j_l3), rmse_display(j_l3));
else
    err_display = err_no_align;
    rmse_display = rmse_no_align;
    mae_display = mae_no_align;
    max_display = max_abs_no_align;
end

fprintf('\n--- 分析一：不进行往返轨迹对齐（时间序列逐点对比） ---\n');
if opts.fit_tau_err_qdd
    fprintf('各关节 RMSE(拟合残差):   '); fprintf(' %.4f', rmse_display); fprintf(' N·m\n');
    fprintf('各关节 MAE(拟合残差):    '); fprintf(' %.4f', mae_display);  fprintf(' N·m\n');
    fprintf('各关节 Max|拟合残差|:    '); fprintf(' %.4f', max_display);  fprintf(' N·m\n');
else
    fprintf('各关节 RMSE(实机-模型):    '); fprintf(' %.4f', rmse_display);    fprintf(' N·m\n');
    if opts.add_friction_line
        rmse_plus_fric = sqrt(mean((tau_real_a - tau_fric_a).^2, 1));
        fprintf('各关节 RMSE(实机-模型+摩擦): '); fprintf(' %.4f', rmse_plus_fric); fprintf(' N·m\n');
    end
    fprintf('各关节 MAE(实机-模型):     '); fprintf(' %.4f', mae_display);     fprintf(' N·m\n');
    fprintf('各关节 Max|实机-模型|:     '); fprintf(' %.4f', max_display); fprintf(' N·m\n');
end

% 图 1：力矩对比 — 两条线（实际、动力学），可选第三条（动力学+摩擦）
fig1 = figure('Name', ['逆动力学_力矩对比_不对齐_', fig_run_tag]);
for j = 1:12
    ax = subplot(3, 4, j);
    hold(ax, 'on');
    plot(ax, time_a, tau_real_a(:, j), 'b-', 'LineWidth', 1, 'DisplayName', '实际关节力矩');
    plot(ax, time_a, tau_model_a(:, j), 'r--', 'LineWidth', 1, 'DisplayName', '动力学');
    if opts.add_friction_line
        plot(ax, time_a, tau_fric_a(:, j), 'g-.', 'LineWidth', 1.2, 'DisplayName', '动力学+摩擦力矩');
    end
    legend(ax, 'Location', 'best');
    xlabel(ax, '时间 (s)'); ylabel(ax, '力矩 (N·m)');
    title(ax, sprintf('%s (RMSE=%.3f)', joint_suffix{j}, rmse_display(j)));
    grid(ax, 'on');
end
sgtitle(fig1, '力矩对比：实际 vs 动力学（不进行往返轨迹对齐）');
save_fig_png(opts, 'torque_compare');

% 图 2：力矩误差（拟合开启时为拟合残差）
figure('Name', ['逆动力学_力矩误差_不对齐_', fig_run_tag]);
for j = 1:12
    subplot(3, 4, j);
    plot(time_a, err_display(:, j), 'k-');
    xlabel('时间 (s)'); ylabel('误差 (N·m)');
    title(sprintf('%s (RMSE=%.3f)', joint_suffix{j}, rmse_display(j)));
    grid on;
end
if opts.fit_tau_err_qdd
    sgtitle('力矩误差（拟合残差）：实机 - 模型 - a·qdd - b');
else
    sgtitle('力矩误差：实机/仿真 - 逆动力学（不进行往返轨迹对齐）');
end
save_fig_png(opts, 'torque_error');

% 图 2b–2d：误差 vs q / qd / qdd（拟合开启时为拟合残差）
figure('Name', ['逆动力学_力矩误差_vs_关节角_', fig_run_tag]);
for j = 1:12
    subplot(3, 4, j);
    plot(q_a(:, j), err_display(:, j), '.', 'MarkerSize', 2);
    xlabel('q (rad)'); ylabel('误差 (N·m)');
    title(joint_suffix{j});
    grid on;
end
if opts.fit_tau_err_qdd
    sgtitle('力矩误差（拟合残差） vs 关节角');
else
    sgtitle('力矩误差 vs 关节角（每子图横轴为该关节角）');
end
save_fig_png(opts, 'error_vs_q');

figure('Name', ['逆动力学_力矩误差_vs_角速度_', fig_run_tag]);
for j = 1:12
    subplot(3, 4, j);
    plot(qd_a(:, j), err_display(:, j), '.', 'MarkerSize', 2);
    xlabel('qd (rad/s)'); ylabel('误差 (N·m)');
    title(joint_suffix{j});
    grid on;
end
if opts.fit_tau_err_qdd
    sgtitle('力矩误差（拟合残差） vs 角速度');
else
    sgtitle('力矩误差 vs 角速度（每子图横轴为该关节角速度）');
end
save_fig_png(opts, 'error_vs_qd');

figure('Name', ['逆动力学_力矩误差_vs_角加速度_', fig_run_tag]);
for j = 1:12
    subplot(3, 4, j);
    plot(qdd_a(:, j), err_display(:, j), '.', 'MarkerSize', 2);
    xlabel('qdd (rad/s^2)'); ylabel('误差 (N·m)');
    title(joint_suffix{j});
    grid on;
end
if opts.fit_tau_err_qdd
    sgtitle('力矩误差（拟合残差） vs 角加速度');
else
    sgtitle('力矩误差 vs 角加速度（每子图横轴为该关节角加速度）');
end
save_fig_png(opts, 'error_vs_qdd');

%% 5. 双文件往返对齐（可选；align=true 且提供 csv_file_reverse 时执行）
% 正向轨迹 = csv_file，逆向轨迹 = csv_file_reverse。将逆向文件逆序后验证：关节角与正向相等、速度相反；
% 再取 (tau_正+tau_反)/2 得到惯性+重力力矩，与模型 ID(q,qd_avg,qdd_avg)=M*qdd+C*qd+G 对比。
if opts.align
    if ~exist(opts.csv_file_reverse, 'file')
        error('未找到逆向轨迹文件: %s', opts.csv_file_reverse);
    end
    data_rev = read_leg_joint_csv(opts.csv_file_reverse);
    time_rev = data_rev.time;
    q_rev    = [data_rev.pos_leg_l, data_rev.pos_leg_r];
    qd_rev   = [data_rev.vel_leg_l, data_rev.vel_leg_r];
    tau_rev  = [data_rev.torque_leg_l, data_rev.torque_leg_r];
    dt_rev   = median(diff(time_rev));
    if dt_rev <= 0 || isnan(dt_rev), dt_rev = dt_default; end
    % 逆向文件 qdd：用表格 qd 中心差分（与主流程一致）
    qdd_rev = zeros(size(qd_rev));
    for j = 1 : 12
        qdd_rev(:, j) = central_diff(qd_rev(:, j), time_rev);
    end
    qdd_rev(~isfinite(qdd_rev)) = 0;

    % 逆向轨迹逆序：行翻转，速度取反（逆序后对应同一路径点则 q 相同、qd 相反）
    q_rev_f   = flip(q_rev, 1);
    qd_rev_f  = -flip(qd_rev, 1);
    tau_rev_f = flip(tau_rev, 1);
    qdd_rev_f = flip(qdd_rev, 1);

    n = min(size(q, 1), size(q_rev_f, 1));
    q_fwd   = q(1:n, :);   qd_fwd   = qd(1:n, :);   qdd_fwd   = qdd(1:n, :);   tau_fwd   = tau_real(1:n, :);
    q_rev_n = q_rev_f(1:n, :); qd_rev_n = qd_rev_f(1:n, :); qdd_rev_n = qdd_rev_f(1:n, :); tau_rev_n = tau_rev_f(1:n, :);

    % 验证：逆序后关节角应相等、速度应相反
    err_q  = max(abs(q_fwd(:) - q_rev_n(:)));
    err_qd = max(abs(qd_fwd(:) + qd_rev_n(:)));  % qd_rev_n = -qd_fwd 则 qd_fwd + qd_rev_n ≈ 0
    fprintf('\n--- 分析二：双文件往返对齐（正向 %d 点，逆向逆序取前 %d 点） ---\n', n, n);
    fprintf('验证：max|q_正 - q_逆序| = %.6f rad，max|qd_正 + qd_逆序| = %.6f rad/s（应为接近 0）\n', err_q, err_qd);
    if err_q > 0.05 || err_qd > 0.5
        warning('正逆轨迹一致性偏差较大，请检查两文件是否为同一路径的正向/逆向录制。');
    end

    % 对齐后对比图：关节角、角速度、角加速度 — 分三张图，正向 vs 逆向逆序
    joint_names_short = {'l1','l2','l3','l4','l5','l6','r1','r2','r3','r4','r5','r6'};
    idx = 1 : n;
    figure('Name', ['双文件往返对齐_关节角_正向vs逆向逆序_', fig_run_tag]);
    for j = 1 : 12
        subplot(3, 4, j);
        plot(idx, q_fwd(:, j), 'b-', 'DisplayName', '正向'); hold on;
        plot(idx, q_rev_n(:, j), 'r--', 'DisplayName', '逆向逆序');
        ylabel('q (rad)'); title(joint_names_short{j}); legend('Location', 'best'); grid on;
    end
    sgtitle('双文件往返对齐：关节角 — 正向 vs 逆向逆序');
    save_fig_png(opts, 'align_q_forward_vs_reverse');

    figure('Name', ['双文件往返对齐_角速度_正向vs逆向逆序_', fig_run_tag]);
    for j = 1 : 12
        subplot(3, 4, j);
        plot(idx, qd_fwd(:, j), 'b-', 'DisplayName', '正向'); hold on;
        plot(idx, qd_rev_n(:, j), 'r--', 'DisplayName', '逆向逆序');
        ylabel('qd (rad/s)'); title(joint_names_short{j}); legend('Location', 'best'); grid on;
    end
    sgtitle('双文件往返对齐：角速度 — 正向 vs 逆向逆序');
    save_fig_png(opts, 'align_qd_forward_vs_reverse');

    figure('Name', ['双文件往返对齐_角加速度_正向vs逆向逆序_', fig_run_tag]);
    for j = 1 : 12
        subplot(3, 4, j);
        plot(idx, qdd_fwd(:, j), 'b-', 'DisplayName', '正向'); hold on;
        plot(idx, qdd_rev_n(:, j), 'r--', 'DisplayName', '逆向逆序');
        xlabel('序号'); ylabel('qdd (rad/s^2)'); title(joint_names_short{j}); legend('Location', 'best'); grid on;
    end
    sgtitle('双文件往返对齐：角加速度 — 正向 vs 逆向逆序');
    save_fig_png(opts, 'align_qdd_forward_vs_reverse');

    tau_aligned = (tau_fwd + tau_rev_n) / 2;
    q_aligned   = q_fwd;
    qd_avg      = (qd_fwd + qd_rev_n) / 2;
    qdd_avg     = (qdd_fwd + qdd_rev_n) / 2;

    % 模型惯性+重力：ID(q,qd_avg,qdd_avg) = M*qdd + C*qd + G
    tau_model_aligned = zeros(n, 12);
    for k = 1 : n
        [tau_L, ~, ~, ~] = compute_e1_limb_dynamics('left_leg',  q_aligned(k, 1:6)',  qd_avg(k, 1:6)',  qdd_avg(k, 1:6)');
        [tau_R, ~, ~, ~] = compute_e1_limb_dynamics('right_leg', q_aligned(k, 7:12)', qd_avg(k, 7:12)', qdd_avg(k, 7:12)');
        tau_model_aligned(k, 1:6)  = tau_L';
        tau_model_aligned(k, 7:12) = tau_R';
    end

    rmse_align = sqrt(mean((tau_aligned - tau_model_aligned).^2, 1));
    joint_names_short = {'l1','l2','l3','l4','l5','l6','r1','r2','r3','r4','r5','r6'};
    % 图：双文件往返对齐 — (tau_正+tau_反)/2 vs 模型惯性+重力
    figure('Name', ['逆动力学_力矩对比_双文件往返对齐_vs_惯性重力_', fig_run_tag]);
    for j = 1 : 12
        subplot(3, 4, j);
        plot(1:n, tau_aligned(:, j), 'b.-', 'DisplayName', '(tau_正+tau_反)/2'); hold on;
        plot(1:n, tau_model_aligned(:, j), 'r.-', 'DisplayName', '模型惯性+重力 Mqdd+Cqd+G');
        xlabel('序号'); ylabel('力矩 (N·m)');
        title(sprintf('%s RMSE=%.3f', joint_names_short{j}, rmse_align(j)));
        legend('Location', 'best');
        grid on;
    end
    sgtitle('双文件往返对齐: (tau_正+tau_反)/2 vs 模型惯性+重力');
    save_fig_png(opts, 'align_vs_inertia_gravity');
    fprintf('各关节 往返对齐-惯性重力 RMSE: '); fprintf(' %.4f', rmse_align); fprintf(' N·m\n');
else
    rmse_align = [];
end

%% 6. 汇总
fprintf('\n===== 汇总 =====\n');
if opts.fit_tau_err_qdd
    fprintf('不对齐 - 全关节平均 RMSE(拟合残差): %.4f N·m\n', mean(rmse_display));
else
    fprintf('不对齐 - 全关节平均 RMSE(数据-模型): %.4f N·m\n', mean(rmse_display));
end
if opts.align && ~isempty(rmse_align)
    fprintf('往返对齐 - 全关节平均 RMSE(惯性+重力): %.4f N·m\n', mean(rmse_align));
    fprintf('说明：双文件正向+逆向取 (tau_正+tau_反)/2 抵消摩擦，与 ID(q,qd_avg,qdd_avg) 惯性+重力对比。\n');
else
    fprintf('未进行往返对齐分析（可传入 ''align'', true, ''csv_file_reverse'', ''path/to/rev.csv'' 启用）。\n');
end
fprintf('\n若数据来自 Gazebo、URDF 一致且均无摩擦，仍「0 速/加速度时误差较大」：\n');
fprintf('  主因：本脚本用 diff(qd)/dt 得到 qdd 再算 ID，而 Gazebo 的力矩由其内部 FD 求解器算出的 qdd 得到；两者 qdd 不一致（数值微分噪声大、尤其 qd≈0 与加速度段），故 ID 输出与 Gazebo 力矩会有差。\n');

% 三个输出均返回（对齐时返回对齐后的时间与序列）
out_data = struct('time', time_a, 'tau_real', tau_real_a, 'tau_model', tau_model_a, 'q', q_a, 'qd', qd_a, 'qdd', qdd_a);
if opts.add_friction_line
    out_data.tau_model_plus_friction = tau_fric_a;
end
varargout{1} = rmse_display;
varargout{2} = rmse_align;
varargout{3} = out_data;
end

% 绘图与信号处理已统一到 utility_function：save_fig_png, lowpass_butter2_zerophase, forward_diff, central_diff, knee_friction_torque
