function extract_target_trajectory_from_csv(input_csv, varargin)
% extract_target_trajectory_from_csv  从录制的CSV中拆取期望轨迹，起止为0位，输出与生成轨迹一致格式
%
% 输入CSV格式：time, 以及每个关节的 target_pos, real_pos, speed, acc, torque
% 关节顺序（CSV中）：l_leg_hip_yaw, r_leg_hip_yaw, l_leg_hip_roll, r_leg_hip_roll,
%   l_leg_hip_pitch, r_leg_hip_pitch, l_leg_knee, r_leg_knee,
%   l_leg_ankle_pitch, r_leg_ankle_pitch, l_leg_ankle_roll, r_leg_ankle_roll
%
% 输出CSV格式：time, leg_l1_joint, leg_l2_joint, ..., leg_r6_joint
% 输出轨迹：起止均为0位姿态（添加两段过渡轨迹）
%
% 用法：
%   extract_target_trajectory_from_csv('recorded_data.csv')
%   extract_target_trajectory_from_csv('recorded_data.csv', 'output_file', 'extracted_trajectory.csv')
%   extract_target_trajectory_from_csv('recorded_data.csv', 'Ts', 0.002, 'transition_time', 1.0)

p = inputParser;
addRequired(p, 'input_csv', @ischar);
addParameter(p, 'output_file', '', @ischar);
addParameter(p, 'Ts', 0.002, @isnumeric);           % 与生成轨迹一致的采样周期
addParameter(p, 'transition_time', 1.0, @isnumeric); % 起止过渡段时长(秒)
parse(p, input_csv, varargin{:});
opts = p.Results;

if ~exist(opts.input_csv, 'file')
    error('输入文件不存在: %s', opts.input_csv);
end

fprintf('===== 从CSV拆取期望轨迹 =====\n');
fprintf('输入文件: %s\n', opts.input_csv);

% 读取表头确定列索引
fid = fopen(opts.input_csv, 'r');
header_line = fgetl(fid);
fclose(fid);
headers = strsplit(header_line, ',');
headers = strtrim(headers);

% 关节在CSV中的顺序（每个关节5列：target_pos, real_pos, speed, acc, torque）
joint_suffix = {'l_leg_hip_yaw_joint', 'r_leg_hip_yaw_joint', 'l_leg_hip_roll_joint', 'r_leg_hip_roll_joint', ...
    'l_leg_hip_pitch_joint', 'r_leg_hip_pitch_joint', 'l_leg_knee_joint', 'r_leg_knee_joint', ...
    'l_leg_ankle_pitch_joint', 'r_leg_ankle_pitch_joint', 'l_leg_ankle_roll_joint', 'r_leg_ankle_roll_joint'};

target_col_idx = [];
for j = 1:length(joint_suffix)
    col_name = [joint_suffix{j} '_target_pos'];
    idx = find(strcmp(headers, col_name));
    if isempty(idx)
        error('未找到列: %s', col_name);
    end
    target_col_idx(j) = idx;
end

% 时间列
time_idx = find(strcmp(headers, 'time'));
if isempty(time_idx)
    time_idx = 1;  % 默认第一列
end

% 读入数据（按列索引读取，避免表头命名差异）
dat = readmatrix(opts.input_csv, 'NumHeaderLines', 1);
if isempty(dat)
    error('CSV无数据行');
end
time = dat(:, time_idx);
% 提取12个关节的 target_pos（按之前解析的列索引）
q_target_raw = dat(:, target_col_idx);

% 重排为输出顺序：左腿1~6，右腿1~6
% CSV顺序: l_yaw, r_yaw, l_roll, r_roll, l_pitch, r_pitch, l_knee, r_knee, l_ankle_pitch, r_ankle_pitch, l_ankle_roll, r_ankle_roll
% 输出顺序: l_yaw, l_roll, l_pitch, l_knee, l_ankle_pitch, l_ankle_roll, r_yaw, r_roll, r_pitch, r_knee, r_ankle_pitch, r_ankle_roll
order_from_csv = [1, 3, 5, 7, 9, 11, 2, 4, 6, 8, 10, 12];
q_center = zeros(length(time), 12);
for j = 1:12
    q_center(:, j) = q_target_raw(:, order_from_csv(j));
end

% 时间统一为从0开始（若需要）
if time(1) ~= 0
    time = time - time(1);
end
dt = median(diff(time));
if dt <= 0 || isnan(dt)
    dt = opts.Ts;
end

% 起止过渡：从0位到第一点、从最后一点到0位（两段轨迹）
zero_pos = zeros(1, 12);
q_first = q_center(1, :);
q_last  = q_center(end, :);

n_trans = max(round(opts.transition_time / opts.Ts), 10);

% 段1：0 -> 第一点（线性插值，与生成脚本中的过渡一致）
t_start = linspace(0, opts.transition_time, n_trans)';
q_start = zeros(n_trans, 12);
for i = 1:n_trans
    alpha = i / n_trans;
    q_start(i, :) = (1 - alpha) * zero_pos + alpha * q_first;
end

% 段2：原始轨迹，统一按 Ts 重采样
t_center_uniform = (0 : opts.Ts : time(end))';
if isempty(t_center_uniform)
    t_center_uniform = 0;
end
q_center_resampled = zeros(length(t_center_uniform), 12);
for j = 1:12
    q_center_resampled(:, j) = interp1(time, q_center(:, j), t_center_uniform, 'linear', 'extrap');
end
t_center = t_center_uniform + opts.transition_time;  % 接在段1之后
q_center = q_center_resampled;

% 段3：最后一点 -> 0（接在段2之后，间隔一个 Ts）
t_end = t_center(end) + opts.Ts + linspace(0, opts.transition_time, n_trans)';
q_end = zeros(n_trans, 12);
for i = 1:n_trans
    if n_trans <= 1
        alpha = 1;
    else
        alpha = (i - 1) / (n_trans - 1);  % 从 q_last 到 0
    end
    q_end(i, :) = (1 - alpha) * q_last + alpha * zero_pos;
end

% 合并：段1 + 段2 + 段3
t_all = [t_start; t_center; t_end];
q_all = [q_start; q_center; q_end];

% 输出表头与生成轨迹一致
out_headers = {'time', 'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
    'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};

if isempty(opts.output_file)
    [~, name, ~] = fileparts(opts.input_csv);
    opts.output_file = fullfile(get_build_dir('plan'), [name '_target_trajectory.csv']);
end

fid = fopen(opts.output_file, 'w');
if fid == -1
    error('无法创建输出文件: %s', opts.output_file);
end
fprintf(fid, '%s\n', strjoin(out_headers, ','));
for i = 1:size(t_all, 1)
    fprintf(fid, '%.5f', t_all(i));
    for j = 1:12
        fprintf(fid, ',%.6f', q_all(i, j));
    end
    fprintf(fid, '\n');
end
fclose(fid);

fprintf('输出文件: %s\n', opts.output_file);
fprintf('总点数: %d, 总时长: %.3f s\n', size(t_all, 1), t_all(end));
fprintf('起止均为0位姿态（已添加两段过渡轨迹）\n');
fprintf('===== 完成 =====\n');
end
