function out_path = read_leg_trajectory_from_csv(input_csv, varargin)
% read_leg_trajectory_from_csv  从 CSV 读取关节数据，仅保留腿部 12 关节并插值到 500Hz
%
% 输入 CSV 格式：
%   - 从第 8 列起为关节数据，列顺序对应：
%     l_leg_hip_yaw, l_leg_hip_roll, l_leg_hip_pitch, l_leg_knee, l_leg_ankle_pitch, l_leg_ankle_roll,
%     r_leg_hip_yaw, r_leg_hip_roll, r_leg_hip_pitch, r_leg_knee, r_leg_ankle_pitch, r_leg_ankle_roll,
%     l_arm_shoulder_pitch, l_arm_shoulder_roll, l_arm_elbow,
%     r_arm_shoulder_pitch, r_arm_shoulder_roll, r_arm_elbow
%   - 仅使用前 12 列关节（双腿），手臂列忽略
%   - 输入采样频率：30 Hz（由参数 fs_in 指定，默认 30）
%   - 若无时间列：默认按 fs_in（30Hz）生成等间隔时间（time_col=0）。
%     若有时间列：传 time_col=1 或对应列号（1-based）。
%
% 输出 CSV 格式（与 generate_e1_test_trajectories 一致）：
%   表头：time, leg_l1_joint, leg_l2_joint, ..., leg_r6_joint
%   数据：每行 time(%.5f) + 12 个关节角 (%.6f)，采样频率 500 Hz
%
% 用法：
%   out_path = read_leg_trajectory_from_csv('recorded_30hz.csv');
%   read_leg_trajectory_from_csv('data.csv', 'output_file', 'leg_500hz.csv');
%   read_leg_trajectory_from_csv('data.csv', 'fs_in', 30, 'fs_out', 500);
%   % 无时间列、30Hz 采样时可直接调用（默认 time_col=0 会按 fs_in 生成时间）：
%   read_leg_trajectory_from_csv('0-251217_run_fastest_跑步CSC_002.csv');
%
% 可选工程参数（建议保留默认）：
%   strict         - true 时时间不单调或列数不足直接报错（默认 true）
%   allow_extrap   - 是否允许插值外推，默认 false（输出时间严格在输入 [t_start,t_end] 内）
%   method         - 插值方法：'pchip'|'linear'|'makima'，默认 'pchip'（比线性更平滑，速度无阶跃）
%   t_start, t_end - 裁剪时间范围 (s)，默认 [] 表示不裁剪（可去掉起步/结尾抖动）
%   demean_time    - 输出时间从 0 开始，默认 true
%   save_mat       - 非空则同时保存 .mat 结构体（traj.t, traj.q_leg, traj.fs, traj.joint_names 等），便于调试

p = inputParser;
addRequired(p, 'input_csv', @ischar);
addParameter(p, 'output_file', '', @ischar);
addParameter(p, 'fs_in', 30, @isnumeric);   % 输入采样频率 (Hz)
addParameter(p, 'fs_out', 500, @isnumeric); % 输出采样频率 (Hz)
addParameter(p, 'time_col', 0, @isnumeric);  % 时间列（1-based）；0 表示无时间列，按 fs_in 生成
addParameter(p, 'strict', true, @islogical);
addParameter(p, 'allow_extrap', false, @islogical);
addParameter(p, 'method', 'pchip', @(x) ischar(x) && ismember(x, {'pchip','linear','makima'}));
addParameter(p, 't_start', [], @(x) isempty(x) || isnumeric(x));
addParameter(p, 't_end', [], @(x) isempty(x) || isnumeric(x));
addParameter(p, 'demean_time', true, @islogical);
addParameter(p, 'save_mat', '', @ischar);
parse(p, input_csv, varargin{:});
opts = p.Results;

if ~exist(opts.input_csv, 'file')
    error('输入文件不存在: %s', opts.input_csv);
end

% 关节列：第 8~19 列对应 12 个腿部关节（与用户给定顺序一致）
leg_col_idx = (8 : 19);  % 1-based
% 输出用关节名（与 generate_e1_test_trajectories 一致）
out_joint_names = {'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
    'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};

fprintf('===== 从 CSV 读取腿部轨迹并插值到 %d Hz =====\n', opts.fs_out);
fprintf('输入文件: %s\n', opts.input_csv);

% 检测分隔符并读取数值
[fid, msg] = fopen(opts.input_csv, 'r');
if fid == -1
    error('无法打开文件: %s (%s)', opts.input_csv, msg);
end
first_line = fgetl(fid);
fclose(fid);
if ~ischar(first_line)
    error('文件为空或无法读取首行: %s', opts.input_csv);
end
% 若首行包含逗号则用逗号，否则用空格
if contains(first_line, ',')
    delim = ',';
else
    delim = ' ';
end

% 尝试带表头读取：若首行为表头则跳过
first_vals = str2num(first_line); %#ok<ST2NM>
if isempty(first_vals) || numel(first_vals) < max(leg_col_idx)
    num_header_lines = 1;
else
    num_header_lines = 0;
end

dat = readmatrix(opts.input_csv, 'NumHeaderLines', num_header_lines, 'Delimiter', delim);
if isempty(dat)
    error('CSV 无有效数据行: %s', opts.input_csv);
end

n_rows = size(dat, 1);
n_cols = size(dat, 2);
if n_cols < max(leg_col_idx)
    error('CSV 列数不足（需要至少 %d 列，当前 %d）', max(leg_col_idx), n_cols);
end

% 时间列
if opts.time_col >= 1 && opts.time_col <= n_cols
    t = dat(:, opts.time_col);
else
    t = (0 : n_rows - 1)' / opts.fs_in;
end
t = t(:);
q_leg = dat(:, leg_col_idx);

% 有效点掩码
mask = isfinite(t) & all(isfinite(q_leg), 2);
t = t(mask);
q_leg = q_leg(mask, :);
if isempty(t)
    error('清洗后无有效时间/关节数据');
end

% 时间去重（stable）
[t, ia] = unique(t, 'stable');
q_leg = q_leg(ia, :);

% 单调性检查（strict 时报错，避免误用 sort 打乱对应关系）
if any(diff(t) <= 0)
    if opts.strict
        error('时间列不单调（存在重复或逆序），请检查源数据。');
    end
    warning('时间列不单调，已忽略异常。');
end

% 可选裁剪 [t_start, t_end]
t_min = t(1);
t_max = t(end);
if ~isempty(opts.t_start) && isnumeric(opts.t_start)
    t_min = max(t_min, opts.t_start);
end
if ~isempty(opts.t_end) && isnumeric(opts.t_end)
    t_max = min(t_max, opts.t_end);
end
idx_trim = t >= t_min & t <= t_max;
t = t(idx_trim);
q_leg = q_leg(idx_trim, :);
if numel(t) < 2
    error('裁剪后时间点不足 2 个');
end

% 输出时间轴：严格在 [t(1), t(end)] 内，不外推
dt_out = 1 / opts.fs_out;
t_new = (t(1) : dt_out : t(end))';
if t_new(end) < t(end) - 1e-9
    t_new = [t_new; t(end)];
end
% 插值：t_new 已在 [t(1),t(end)] 内，不触发外推；仅当 allow_extrap 时对边界外点外推（一般不需要）
q_out = zeros(numel(t_new), 12);
for j = 1 : 12
    if opts.allow_extrap
        q_out(:, j) = interp1(t, q_leg(:, j), t_new, opts.method, 'extrap');
    else
        q_out(:, j) = interp1(t, q_leg(:, j), t_new, opts.method);
    end
end

% 时间归零（便于对齐）
if opts.demean_time
    t0 = t_new(1);
    t_new = t_new - t0;
end

% 默认输出路径
if isempty(opts.output_file)
    base_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(base_dir, '..');
    addpath(fullfile(repo_root, 'utility_function'));
    [~, name, ~] = fileparts(opts.input_csv);
    opts.output_file = fullfile(get_build_dir('plan'), [name '_leg_500Hz.csv']);
end
out_dir = fileparts(opts.output_file);
if ~isempty(out_dir) && ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% 按 generate_e1_test_trajectories 格式写 CSV
fid = fopen(opts.output_file, 'w');
if fid == -1
    error('无法创建输出文件: %s', opts.output_file);
end
header_cells = [{'time'}, out_joint_names];
for c = 1 : numel(header_cells)
    if c < numel(header_cells)
        fprintf(fid, '%s,', header_cells{c});
    else
        fprintf(fid, '%s\n', header_cells{c});
    end
end
for i = 1 : numel(t_new)
    fprintf(fid, '%.5f', t_new(i));
    for j = 1 : 12
        fprintf(fid, ',%.6f', q_out(i, j));
    end
    fprintf(fid, '\n');
end
fclose(fid);

out_path = opts.output_file;
fprintf('输出文件: %s\n', out_path);
fprintf('输入: %d 点 (%.3f s) -> 输出: %d 点 @ %d Hz (%.3f s)\n', ...
    n_rows, t_max - t_min, numel(t_new), opts.fs_out, t_new(end) - t_new(1));
fprintf('===== 完成 =====\n');

% 可选：保存 .mat 结构体（调试/中间缓存；含插值前后数据便于绘图）
if ~isempty(opts.save_mat)
    traj = struct();
    traj.t = t_new;
    traj.q_leg = q_out;
    traj.fs = opts.fs_out;
    traj.joint_names = out_joint_names;
    dt_out = 1 / opts.fs_out;
    traj.qd = [zeros(1, 12); diff(q_out) / dt_out];
    traj.qdd = [zeros(1, 12); diff(traj.qd) / dt_out];
    traj.t_raw = t - t(1);      % 插值前时间（归零，与 traj.t 对齐）
    traj.q_leg_raw = q_leg;     % 插值前 12 关节
    save(opts.save_mat, 'traj');
    fprintf('已保存 MAT: %s (含 traj.t/t_raw, traj.q_leg/q_leg_raw, traj.qd, traj.qdd, traj.fs, traj.joint_names)\n', opts.save_mat);
end
end
