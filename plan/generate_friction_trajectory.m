% generate_friction_trajectory  生成摩擦力辨识轨迹（每关节两文件：速度/加速度）
%
% 功能：从URDF读取关节最大速度，为每个关节单独生成两个CSV文件
%       速度轨迹包含正、负两个方向（正速与负速档位），用于摩擦辨识分别拟合 τ_c+b 与 -τ_c+b
%       1. 速度轨迹与加速度轨迹分开放置
%       2. 每个关节：{关节名}_velocity_trajectory_500Hz.csv、{关节名}_acceleration_trajectory_500Hz.csv
%       格式均为：time + 12个关节列；500Hz；MOVEJ无突变
%       其余关节由 other_joints_mode 决定：'zero' 时为 0，'reference' 时与参考文件对应列一致
%
% 输入参数（可选）:
%   varargin - 可选参数（名值对）:
%     'joint_names' - 要生成的关节名（cell），默认仅 'l_leg_hip_yaw_joint'
%     'urdf_file'   - URDF文件路径，默认自动查找
%     'a_max'       - 关节最大加速度 rad/s²，默认 30.0
%     'output_dir'  - 输出目录，默认当前目录
%     'do_plot'     - 是否绘制轨迹预览图，默认 true
%     'other_joints_mode' - 其余关节模式：'zero' 除当前关节外其余为 0；
%                          'reference' 除当前关节外与参考文件中对应列保持一致。默认 'zero'
%     'reference_trajectory_file' - 参考轨迹 CSV 路径（time + 12 关节列），
%                                  仅当 other_joints_mode='reference' 时必需
%     'q_limit' - 关节位置限位 ±q_limit (rad)，轨迹将裁剪在该范围内；默认 12.5；[] 表示不限制
%
% 输出:
%   每个关节在 output_dir 下生成两个文件：
%   - {关节名}_velocity_trajectory_500Hz.csv
%   - {关节名}_acceleration_trajectory_500Hz.csv
%   表头：time,l_leg_hip_yaw_joint,l_leg_hip_roll_joint,...,r_leg_ankle_roll_joint
%   采样 500 Hz；关节角、速度、加速度无突变

function generate_friction_trajectory(varargin)

Ts = 1/500;  % 采样周期 0.002 s，500 Hz

csv_joint_names = {
    'l_leg_hip_yaw_joint';
    'l_leg_hip_roll_joint';
    'l_leg_hip_pitch_joint';
    'l_leg_knee_joint';
    'l_leg_ankle_pitch_joint';
    'l_leg_ankle_roll_joint';
    'r_leg_hip_yaw_joint';
    'r_leg_hip_roll_joint';
    'r_leg_hip_pitch_joint';
    'r_leg_knee_joint';
    'r_leg_ankle_pitch_joint';
    'r_leg_ankle_roll_joint'
};

p = inputParser;
addParameter(p, 'joint_names', {'l_leg_hip_yaw_joint'}, @iscell);
addParameter(p, 'urdf_file', '', @ischar);
addParameter(p, 'a_max', 30.0, @isnumeric);
addParameter(p, 'output_dir', '', @ischar);
addParameter(p, 'do_plot', true, @islogical);
addParameter(p, 'other_joints_mode', 'zero', @(x) ischar(x) && ismember(x, {'zero', 'reference'}));
addParameter(p, 'reference_trajectory_file', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'q_limit', 12.5, @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));  % 关节位置限位 ±q_limit (rad)，[] 表示不限制
parse(p, varargin{:});
opts = p.Results;
if isstring(opts.reference_trajectory_file)
    opts.reference_trajectory_file = char(opts.reference_trajectory_file);
end

fprintf('===== 生成摩擦力辨识轨迹（每关节两文件：速度/加速度，500Hz） =====\n');
fprintf('待生成关节: %s\n', strjoin(opts.joint_names, ', '));
fprintf('采样频率: 500 Hz (Ts = %.4f s)\n', Ts);
fprintf('最大加速度: %.2f rad/s²\n', opts.a_max);
if strcmp(opts.other_joints_mode, 'zero')
    fprintf('其余关节模式: 除当前关节外为 0\n');
else
    fprintf('其余关节模式: 与参考轨迹对应列保持一致\n');
    if isempty(opts.reference_trajectory_file)
        error('other_joints_mode 为 ''reference'' 时须指定 reference_trajectory_file');
    end
    if ~exist(opts.reference_trajectory_file, 'file')
        error('参考轨迹文件不存在: %s', opts.reference_trajectory_file);
    end
end

% 默认 URDF
if isempty(opts.urdf_file)
    default_urdf = fullfile('noetix_description', 'urdf', 'E1.urdf');
    if exist(default_urdf, 'file')
        opts.urdf_file = default_urdf;
    else
        cur_dir = fileparts(mfilename('fullpath'));
        default_urdf = fullfile(cur_dir, '..', '..', 'noetix_description', 'urdf', 'E1.urdf');
        if exist(default_urdf, 'file')
            opts.urdf_file = default_urdf;
        end
    end
end
if isempty(opts.urdf_file) || ~exist(opts.urdf_file, 'file')
    error('未找到URDF文件，请指定 urdf_file');
end

if isempty(opts.output_dir)
    opts.output_dir = get_build_dir('plan');
end
if ~exist(opts.output_dir, 'dir')
    mkdir(opts.output_dir);
end

% 参考轨迹（仅 other_joints_mode == 'reference' 时加载）
ref_t = [];
ref_Q = [];
if strcmp(opts.other_joints_mode, 'reference')
    ref_data = readmatrix(opts.reference_trajectory_file, 'NumHeaderLines', 1);
    if size(ref_data, 2) < 13
        error('参考轨迹文件须包含 time + 12 关节列，当前列数: %d', size(ref_data, 2));
    end
    ref_t = ref_data(:, 1);
    ref_Q = ref_data(:, 2:13);  % 12 列关节
    fprintf('已加载参考轨迹: %s (%d 点, %.2f ~ %.2f s)\n', ...
        opts.reference_trajectory_file, length(ref_t), ref_t(1), ref_t(end));
end

%% 添加路径（摩擦力与规划均通过 addpaths 统一加载 robot_algorithm 与 plan）
% 若已执行 addpaths，无需单独 addpath
cur_dir = fileparts(mfilename('fullpath'));
robot_path = fullfile(cur_dir, '..', 'robot_algorithm');
if exist(robot_path, 'dir') && isempty(which('identify_joint_friction'))
    addpath(genpath(robot_path));
end

%% 从 URDF 读取 12 个关节的 velocity 限位
fprintf('\n========== 从URDF读取关节最大速度 ==========\n');
joint_name_mapping = containers.Map({
    'l_leg_hip_yaw_joint'; 'l_leg_hip_roll_joint'; 'l_leg_hip_pitch_joint';
    'l_leg_knee_joint'; 'l_leg_ankle_pitch_joint'; 'l_leg_ankle_roll_joint';
    'r_leg_hip_yaw_joint'; 'r_leg_hip_roll_joint'; 'r_leg_hip_pitch_joint';
    'r_leg_knee_joint'; 'r_leg_ankle_pitch_joint'; 'r_leg_ankle_roll_joint'
}, {
    'leg_l1_joint'; 'leg_l2_joint'; 'leg_l3_joint';
    'leg_l4_joint'; 'leg_l5_joint'; 'leg_l6_joint';
    'leg_r1_joint'; 'leg_r2_joint'; 'leg_r3_joint';
    'leg_r4_joint'; 'leg_r5_joint'; 'leg_r6_joint'
});

xmlDoc = xmlread(opts.urdf_file);
jointNodes = xmlDoc.getElementsByTagName('joint');
numJoints = jointNodes.getLength();

joint_v_max = zeros(12, 1);
for i = 1:12
    jname = csv_joint_names{i};
    urdf_name = joint_name_mapping(jname);
    v_max_i = 4.0;
    for j = 0:numJoints - 1
        node = jointNodes.item(j);
        if ~strcmp(char(node.getAttribute('name')), urdf_name), continue; end
        limit = node.getElementsByTagName('limit').item(0);
        if isempty(limit), break; end
        v = str2double(limit.getAttribute('velocity'));
        if ~isnan(v) && v > 0
            v_max_i = v;
        end
        break;
    end
    joint_v_max(i) = v_max_i;
    fprintf('  %s: v_max=%.2f rad/s\n', jname, v_max_i);
end

%% 表头（所有文件统一）
header = 'time';
for k = 1:length(csv_joint_names)
    header = [header ',' csv_joint_names{k}];
end

%% 仅对指定关节写两个文件：速度、加速度
for k = 1:length(opts.joint_names)
    jname = opts.joint_names{k};
    j = find(strcmp(csv_joint_names, jname), 1);
    if isempty(j)
        warning('未知关节名: %s，跳过', jname);
        continue;
    end
    v_max = joint_v_max(j);
    fprintf('\n---------- 关节 %d/%d: %s (v_max=%.2f) ----------\n', k, length(opts.joint_names), jname, v_max);

    % ----- 1) 速度轨迹 -> 仅写 CSV，不生成 .mat -----
    trajectory = generate_joint_trajectory('velocity', 'Ts', Ts, 'v_max', v_max, ...
        'output_format', 'mat', 'output_file', '', 'do_plot', opts.do_plot, 'q_limit', opts.q_limit);
    t_vel = trajectory.time(:);
    q_vel = trajectory.position(:);

    if isempty(t_vel) || length(t_vel) < 2
        warning('关节 %s 速度轨迹为空，跳过', jname);
    else
        % 仅当存在重复或未排序时才去重（保证 interp1 不报错；不重采样，行数=轨迹点数）
        if ~issorted(t_vel) || any(diff(t_vel(:)) <= 0)
            [t_vel, sidx] = sort(t_vel(:));
            q_vel = q_vel(sidx);
            [t_vel, iu] = unique(t_vel, 'last');
            q_vel = q_vel(iu);
        end
        if length(t_vel) < 2
            warning('关节 %s 速度轨迹去重后不足2点，跳过', jname);
        else
            M_vel = fill_joint_matrix(t_vel, q_vel, j, opts.other_joints_mode, ref_t, ref_Q);
            vel_path = fullfile(opts.output_dir, [jname '_velocity_trajectory_500Hz.csv']);
            write_csv_12cols(vel_path, header, t_vel, M_vel);
            fprintf('  已写: %s (%d 点, %.2f s)\n', vel_path, length(t_vel), t_vel(end));
        end
    end

    % ----- 2) 加速度轨迹 -> 仅写 CSV -----
    trajectory = generate_joint_trajectory('acceleration', 'Ts', Ts, 'v_max', v_max, ...
        'a_max', opts.a_max, 'output_format', 'mat', 'output_file', '', 'do_plot', opts.do_plot, 'q_limit', opts.q_limit);
    t_acc = trajectory.time(:);
    q_acc = trajectory.position(:);

    if isempty(t_acc) || length(t_acc) < 2
        warning('关节 %s 加速度轨迹为空，跳过', jname);
    else
        if ~issorted(t_acc) || any(diff(t_acc(:)) <= 0)
            [t_acc, sidx] = sort(t_acc(:));
            q_acc = q_acc(sidx);
            [t_acc, iu] = unique(t_acc, 'last');
            q_acc = q_acc(iu);
        end
        if length(t_acc) < 2
            warning('关节 %s 加速度轨迹去重后不足2点，跳过', jname);
        else
            M_acc = fill_joint_matrix(t_acc, q_acc, j, opts.other_joints_mode, ref_t, ref_Q);
            acc_path = fullfile(opts.output_dir, [jname '_acceleration_trajectory_500Hz.csv']);
            write_csv_12cols(acc_path, header, t_acc, M_acc);
            fprintf('  已写: %s (%d 点, %.2f s)\n', acc_path, length(t_acc), t_acc(end));
        end
    end
end

fprintf('\n===== 轨迹生成完成 =====\n');
fprintf('输出目录: %s\n', opts.output_dir);
fprintf('已生成关节: %s\n', strjoin(opts.joint_names, ', '));
fprintf('每个关节 2 个文件：*_velocity_trajectory_500Hz.csv, *_acceleration_trajectory_500Hz.csv\n');
fprintf('采样频率: 500 Hz\n');

end

function M = fill_joint_matrix(t, q_current, j, other_joints_mode, ref_t, ref_Q)
% 构建 n x 12 关节矩阵：第 j 列为当前关节轨迹 q_current，其余列按模式填充
% other_joints_mode: 'zero' 其余为 0；'reference' 其余列按时间插值自 ref_t, ref_Q
n = length(t);
M = zeros(n, 12);
M(:, j) = q_current(:);
if strcmp(other_joints_mode, 'reference') && ~isempty(ref_t) && ~isempty(ref_Q)
    for c = 1:12
        if c == j, continue; end
        M(:, c) = interp1(ref_t, ref_Q(:, c), t(:), 'linear', 'extrap');
    end
end
end

function write_csv_12cols(filepath, header, t, Q)
% Q: n x 12，批量写入避免逐行 fprintf 过慢
fid = fopen(filepath, 'w');
if fid == -1
    error('无法创建文件: %s', filepath);
end
fprintf(fid, '%s\n', header);
fclose(fid);
% 一次性写入数值（比逐行 fprintf 快很多）
writematrix([t, Q], filepath, 'WriteMode', 'append', 'Delimiter', ',');
end
