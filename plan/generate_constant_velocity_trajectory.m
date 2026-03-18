% generate_constant_velocity_trajectory  生成单关节匀速轨迹（格式与摩擦力辨识 CSV 一致）
%
% 功能：生成指定速度、持续时间的匀速轨迹，CSV 格式与 generate_friction_trajectory 相同
%       表头：time + 12 个关节列；仅指定关节列非零；200 Hz
%
% 输入参数（可选）:
%   varargin - 可选参数（名值对）:
%     'velocity'    - 匀速速度 rad/s，默认 4.0
%     'duration'    - 持续时间 s，默认 10.0
%     'joint_name'  - 关节名，默认 'l_leg_hip_yaw_joint'
%     'output_dir'  - 输出目录，默认当前目录
%     'output_file' - 输出文件名，默认 'constant_velocity_4rads_10s.csv'
%
% 示例:
%   generate_constant_velocity_trajectory();
%   generate_constant_velocity_trajectory('velocity', 4, 'duration', 10, 'joint_name', 'l_leg_hip_yaw_joint');

function generate_constant_velocity_trajectory(varargin)

Ts = 1/200;  % 200 Hz，与之前一致

csv_joint_names = {
    'l_leg_hip_yaw_joint'; 'l_leg_hip_roll_joint'; 'l_leg_hip_pitch_joint';
    'l_leg_knee_joint'; 'l_leg_ankle_pitch_joint'; 'l_leg_ankle_roll_joint';
    'r_leg_hip_yaw_joint'; 'r_leg_hip_roll_joint'; 'r_leg_hip_pitch_joint';
    'r_leg_knee_joint'; 'r_leg_ankle_pitch_joint'; 'r_leg_ankle_roll_joint'
};

% URDF关节名映射（CSV关节名 -> URDF关节名）
urdf_joint_mapping = containers.Map({
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

p = inputParser;
addParameter(p, 'velocity', 4.0, @isnumeric);
addParameter(p, 'duration', [], @isnumeric);  % 如果为空，根据URDF限位自动计算
addParameter(p, 'joint_name', 'l_leg_hip_yaw_joint', @ischar);
addParameter(p, 'urdf_file', '', @ischar);
addParameter(p, 'output_dir', '', @ischar);
addParameter(p, 'output_file', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

if isempty(opts.output_dir)
    opts.output_dir = get_build_dir('plan');
end

j = find(strcmp(csv_joint_names, opts.joint_name), 1);
if isempty(j)
    error('未知关节名: %s', opts.joint_name);
end

% 从URDF读取关节限位
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

lower_limit = -pi;  % 默认值
upper_limit = pi;   % 默认值
if ~isempty(opts.urdf_file) && exist(opts.urdf_file, 'file')
    try
        urdf_joint_name = urdf_joint_mapping(opts.joint_name);
        xmlDoc = xmlread(opts.urdf_file);
        jointNodes = xmlDoc.getElementsByTagName('joint');
        numJoints = jointNodes.getLength();
        
        found_joint = false;
        for i = 0:numJoints - 1
            jointNode = jointNodes.item(i);
            jointNameAttr = char(jointNode.getAttribute('name'));
            
            if strcmp(jointNameAttr, urdf_joint_name)
                limitElement = jointNode.getElementsByTagName('limit').item(0);
                if ~isempty(limitElement)
                    lower_str = char(limitElement.getAttribute('lower'));
                    upper_str = char(limitElement.getAttribute('upper'));
                    lower_limit = str2double(lower_str);
                    upper_limit = str2double(upper_str);
                    
                    if ~isnan(lower_limit) && ~isnan(upper_limit)
                        found_joint = true;
                        fprintf('从URDF读取关节限位: %s (%s)\n', opts.joint_name, urdf_joint_name);
                        fprintf('  lower: %.4f rad, upper: %.4f rad\n', lower_limit, upper_limit);
                    else
                        warning('关节 %s (%s) 的limit标签存在，但lower或upper值为NaN (lower="%s", upper="%s")', ...
                            opts.joint_name, urdf_joint_name, lower_str, upper_str);
                    end
                else
                    warning('关节 %s (%s) 没有limit标签', opts.joint_name, urdf_joint_name);
                end
                break;
            end
        end
        
        if ~found_joint
            warning('在URDF中未找到关节 %s，使用默认限位', urdf_joint_name);
        end
    catch ME
        warning('读取URDF失败: %s，使用默认限位', ME.message);
    end
end

% 计算运动范围（限位的0.9倍，与generate_urdf_validation_trajectory一致）
% 但确保在限位范围内：从限位的5%到95%
q_safe_lower = lower_limit + (upper_limit - lower_limit) * 0.05;
q_safe_upper = upper_limit - (upper_limit - lower_limit) * 0.05;
range = q_safe_upper - q_safe_lower;  % 实际可用范围

% 从0开始运动（与generate_urdf_validation_trajectory的Part A一致）
% 但如果0不在安全范围内，则从安全范围的中心开始
q_center = (q_safe_lower + q_safe_upper) / 2;
if q_safe_lower <= 0 && 0 <= q_safe_upper
    q_start = 0;
else
    q_start = q_center;
end
q_end = q_start + range;  % 正转到q_start + range
% 确保不超过安全上限
q_end = min(q_end, q_safe_upper);

fprintf('运动范围: %.4f rad (限位的90%%)\n', range);
fprintf('安全范围: [%.4f, %.4f] rad\n', q_safe_lower, q_safe_upper);
fprintf('起始位置: %.4f rad, 正转结束位置: %.4f rad\n', q_start, q_end);

% 如果duration未指定，根据速度和运动范围自动计算
if isempty(opts.duration)
    opts.duration = range / abs(opts.velocity);
    fprintf('自动计算持续时间: %.2f s (范围 %.4f rad / 速度 %.2f rad/s)\n', ...
        opts.duration, range, opts.velocity);
end

if isempty(opts.output_file)
    opts.output_file = fullfile(opts.output_dir, ...
        sprintf('constant_velocity_%.1frads_%.0fs.csv', opts.velocity, opts.duration));
end

% 生成往返轨迹（正反转匀速，与generate_urdf_validation_trajectory的Part A方式一致）
% 段1：正转（从q_start到q_end）
actual_range_forward = q_end - q_start;  % 实际正转范围
t_forward = actual_range_forward / abs(opts.velocity);
n_forward = round(t_forward / Ts);
t_forward_vec = (0:(n_forward-1))' * Ts;
q_forward = q_start + opts.velocity * t_forward_vec;
% 确保不超过结束位置
q_forward = min(q_forward, q_end);

% 段2：回程（从q_end回到q_start，不参与计算，但需要记录）
t_return = t_forward;
n_return = round(t_return / Ts);
t_return_vec = t_forward_vec(end) + Ts + (0:(n_return-1))' * Ts;
q_return = q_end - opts.velocity * (t_return_vec - t_return_vec(1));

% 段3：反转（从q_start到q_start - range，即负方向）
q_reverse_start = q_start;
q_reverse_end = q_start - range;  % 反转到q_start - range
% 确保不超过安全下限
q_reverse_end = max(q_reverse_end, q_safe_lower);
reverse_range = abs(q_reverse_start - q_reverse_end);
t_reverse = reverse_range / abs(opts.velocity);
n_reverse = round(t_reverse / Ts);
if n_reverse > 0
    t_reverse_vec = t_return_vec(end) + Ts + (0:(n_reverse-1))' * Ts;
    q_reverse = q_reverse_start - opts.velocity * (t_reverse_vec - t_reverse_vec(1));
    % 确保不超过结束位置
    q_reverse = max(q_reverse, q_reverse_end);
else
    t_reverse_vec = [];
    q_reverse = [];
    t_reverse = 0;
    n_reverse = 0;
end

% 合并所有段
t = [t_forward_vec; t_return_vec; t_reverse_vec];
q = [q_forward; q_return; q_reverse];

fprintf('正转段: %.2f s, %d 点 (%.4f -> %.4f rad)\n', t_forward, n_forward, q_start, q_end);
fprintf('回程段: %.2f s, %d 点 (%.4f -> %.4f rad，不参与计算)\n', t_return, n_return, q_end, q_start);
fprintf('反转段: %.2f s, %d 点 (%.4f -> %.4f rad)\n', t_reverse, n_reverse, q_reverse_start, q_reverse_end);
fprintf('总时长: %.2f s, 总数据点数: %d\n', t(end), length(t));

% 表头与 12 列（仅第 j 列非零）
header = 'time';
for k = 1:length(csv_joint_names)
    header = [header ',' csv_joint_names{k}];
end
Q = zeros(length(t), 12);
Q(:, j) = q;

% 写入 CSV（与 generate_friction_trajectory 相同格式）
if contains(opts.output_file, filesep) || (ispc && contains(opts.output_file, '/'))
    out_path = opts.output_file;
else
    out_path = fullfile(opts.output_dir, opts.output_file);
end
fid = fopen(out_path, 'w');
if fid == -1
    error('无法创建文件: %s', out_path);
end
fprintf(fid, '%s\n', header);
fclose(fid);
writematrix([t, Q], out_path, 'WriteMode', 'append', 'Delimiter', ',');

fprintf('\n已生成往返匀速轨迹: %s\n', out_path);
fprintf('  速度: %.1f rad/s, 采样: 200 Hz\n', opts.velocity);
fprintf('  运动范围: %.4f rad (URDF限位的90%%)\n', range);
end
