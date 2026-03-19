%% plot_preprocess_before_after_trajectories
% 绘制“数据预处理前/后”的关节轨迹对比（q/qd/qdd/tau）。
%
% 对比定义：
%   - 预处理前（use_preprocess_id=false）：
%       q_before  = CSV 的 q
%       qd_before = CSV 的 qd
%       qdd_before= 从 qd 的中心差分得到的 qdd（与 run_min_param_id_from_csv 一致）
%       tau_before= CSV 的 tau
%   - 预处理后（use_preprocess_id=true）：
%       q_after   = preprocess_id_data 输出的 q_s（SG 平滑后 q）
%       qd_after  = preprocess_id_data 输出的 qd_s（SG 求导后 qd）
%       qdd_after = preprocess_id_data 输出的 qdd_s（SG 求二阶导后 qdd）
%       tau_after（两个口径）：
%           aux.tau_s  = Butterworth 零相位低通后的 tau_s
%           tau_id     = preprocess_id_data 输出的辨识目标 tau_id
%
% 用法：
%   在 MATLAB 中运行本脚本；必要时修改顶部的 csv_file / limb / 时间窗与滤波参数。

clc; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 0) 配置区
% csv_file = '' 时：按 run_min_param_id_from_csv 的逻辑选最新一个 CSV
csv_file = '';

limb = 'left_leg';  % 目前仅建议用 left_leg（read_leg_joint_csv 已按 left/right 组织）

% 参与绘图的关节（1..6）
plot_joints = 1:6;

% 预处理时间窗（建议与 run_min_param_id_from_csv 的 prep_opts 一致）
prep_opts = struct();
prep_opts.t_start_s = 2.1;
prep_opts.t_end_s   = 4.1;

% SG 求导参数
prep_opts.sg_order = 4;
prep_opts.sg_frame = 23;

% tau 低通参数（零相位）
prep_opts.tau_lowpass_fc_Hz = 15;
prep_opts.tau_lowpass_order = 2;

% τ_id 的“补偿项”开关：
%   - 若你只想看 SG + 低通效果，可以保持为空（默认等价于 tau_id ~ tau_s）
prep_opts.J_eq = [];
prep_opts.friction_params = [];
prep_opts.tau_bias = [];

% 关闭 preprocess 内部绘图（我们自己画对比图）
prep_opts.do_plot = false;
prep_opts.plot_joints = plot_joints;

% 中间差分（预处理前）对齐用：与 run_min_param_id_from_csv 一致
use_central_diff = true;

% 保存图（可选）
save_png = false;
out_dir = fullfile(app_root, '..', 'fig_png');
if ~isfolder(out_dir), mkdir(out_dir); end

%% 1) 选择并读取 CSV
if isstring(csv_file), csv_file = char(csv_file); end
if isempty(csv_file)
    dir1 = fullfile(app_root, 'data', 'excitation');
    dir2 = fullfile(app_root, 'data', '跑步');
    if isfolder(dir1)
        f = dir(fullfile(dir1, '*.csv'));
        if ~isempty(f), [~, idx] = sort([f.datenum], 'descend'); f = f(idx); csv_file = fullfile(dir1, f(1).name); end
    end
    if isempty(csv_file) && isfolder(dir2)
        f = dir(fullfile(dir2, '*.csv'));
        if ~isempty(f), [~, idx] = sort([f.datenum], 'descend'); f = f(idx); csv_file = fullfile(dir2, f(1).name); end
    end
    if isempty(csv_file)
        error('plot_preprocess_before_after_trajectories: 未找到 CSV 文件。请手动设置 csv_file。');
    end
end

if exist(csv_file, 'file') ~= 2
    error('plot_preprocess_before_after_trajectories: 文件不存在: %s', csv_file);
end

fprintf('CSV: %s\n', csv_file);

data = read_leg_joint_csv(csv_file);
t = data.time(:);
q   = data.pos_leg_l;    % N×6
qd  = data.vel_leg_l;    % N×6
tau = data.torque_leg_l; % N×6

N = length(t);
if size(q,1) ~= N || size(tau,1) ~= N
    error('plot_preprocess_before_after_trajectories: t/q/tau 行数不一致。');
end

%% 2) 预处理前：中心差分 qdd + 原始 tau（并裁到同一时间窗）
dt = median(diff(t));
if dt <= 0 || isnan(dt), dt = 0.002; end

if ~use_central_diff
    error('plot_preprocess_before_after_trajectories: 目前仅实现中心差分得到 qdd_before。');
end

qdd_before_full = zeros(N, 6);
for j = 1:6
    for k = 1:N
        qdd_before_full(k, j) = central_diff_point(qd(:, j), t, k, dt);
    end
end

% 与 preprocess_id_data 的 bad 行剔除一致：preprocess 默认剔除非有限 q 或 tau
idx_win = true(N, 1);
if ~isempty(prep_opts.t_start_s), idx_win = idx_win & (t >= prep_opts.t_start_s); end
if ~isempty(prep_opts.t_end_s),   idx_win = idx_win & (t <= prep_opts.t_end_s); end
bad = any(~isfinite(q), 2) | any(~isfinite(tau), 2);
idx_win = idx_win & ~bad;

t_before = t(idx_win);
q_before = q(idx_win, :);
qd_before= qd(idx_win, :);
qdd_before = qdd_before_full(idx_win, :);
tau_before = tau(idx_win, :);

%% 3) 预处理后：调用 preprocess_id_data
[t_after, q_after, qd_after, qdd_after, tau_id_after, aux] = preprocess_id_data(t, q, qd, tau, prep_opts);
tau_s_after = aux.tau_s;

fprintf('before: M=%d, t=[%.3f, %.3f]\n', numel(t_before), t_before(1), t_before(end));
fprintf('after : M=%d, t=[%.3f, %.3f]\n', numel(t_after), t_after(1), t_after(end));

if numel(t_before) ~= numel(t_after)
    fprintf('注意：preprocess 后裁剪/剔除行导致长度不等（before=%d, after=%d）。图仍会分别绘制。\n', numel(t_before), numel(t_after));
end

% 防止 plot_joints 越界
plot_joints = plot_joints(plot_joints >= 1 & plot_joints <= 6);
if isempty(plot_joints), plot_joints = 1; end

nj = numel(plot_joints);

%% 4) 绘图：q/qd/qdd/tau 四行对比（列=关节）
figure('Name', 'Preprocess_Before_After_trajectories', 'Position', [60, 60, 1500, 900]);
for ii = 1:nj
    j = plot_joints(ii);

    % q
    subplot(4, nj, ii);
    plot(t_before, q_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  q_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('q (rad)'); title(sprintf('q joint%d', j));
    legend({'before (raw)', 'after (SG)'}, 'Location', 'best'); grid on; hold off;

    % qd
    subplot(4, nj, nj + ii);
    plot(t_before, qd_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  qd_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('qd (rad/s)'); title(sprintf('qd joint%d', j));
    legend({'before (CSV)', 'after (SG)'}, 'Location', 'best'); grid on; hold off;

    % qdd
    subplot(4, nj, 2*nj + ii);
    plot(t_before, qdd_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  qdd_after(:, j),  'b', 'LineWidth', 1.1);
    xlabel('t (s)'); ylabel('qdd (rad/s^2)'); title(sprintf('qdd joint%d', j));
    legend({'before (central diff)', 'after (SG 2nd)'}, 'Location', 'best'); grid on; hold off;

    % tau
    subplot(4, nj, 3*nj + ii);
    plot(t_before, tau_before(:, j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_after,  tau_s_after(:, j), 'b', 'LineWidth', 1.1);
    plot(t_after,  tau_id_after(:, j), 'k--', 'LineWidth', 1.0);
    xlabel('t (s)'); ylabel('\tau (N\cdotm)'); title(sprintf('\\tau joint%d', j));
    legend({'before (CSV)', 'after (tau\_s lowpass)', 'after (tau\_id target)'}, 'Location', 'best');
    grid on; hold off;
end

sgtitle(sprintf('预处理前后轨迹对比（%s, t=[%.2f, %.2f], SG[%d,%d], tau lowpass=%dHz）',...
    limb, prep_opts.t_start_s, prep_opts.t_end_s, prep_opts.sg_order, prep_opts.sg_frame, prep_opts.tau_lowpass_fc_Hz));

%% 5) 可选保存
if save_png
    out_name = sprintf('preprocess_before_after_%s_t%.2fto%.2f_SG%d_%dHz.png', ...
        limb, prep_opts.t_start_s, prep_opts.t_end_s, prep_opts.sg_order, prep_opts.tau_lowpass_fc_Hz);
    out_path = fullfile(out_dir, out_name);
    fprintf('保存图: %s\n', out_path);
    saveas(gcf, out_path);
end

fprintf('完成：已绘制 q/qd/qdd/\\tau 的预处理前后对比。\n');

