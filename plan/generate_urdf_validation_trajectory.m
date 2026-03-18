function generate_urdf_validation_trajectory(trajectory_type, joint_name, varargin)
% generate_urdf_validation_trajectory  生成URDF动力学参数验证测试轨迹
%
% 功能：根据测试方案生成用于验证URDF动力学参数的测试轨迹
%       Part A: 正反转匀速轨迹（用于重力验证）
%       Part B: 带加速度轨迹（用于完整动力学力矩验证）
%
% 输入参数:
%   trajectory_type - 轨迹类型：
%                     'part_a'  : Part A轨迹（正反转匀速）
%                     'part_b'  : Part B轨迹（带加速度）
%   joint_name     - 要测试的关节名称（如 'l_leg_hip_yaw_joint'）
%   varargin - 可选参数（名值对）:
%     'Ts'              - 采样周期，默认 0.002 s (500 Hz)
%     'velocity'        - 匀速速度 rad/s（Part A用），默认 0.2
%     'angle_range'     - 角度范围 rad（Part A用），默认 2*pi（1圈）
%     'trajectory_shape'- Part B轨迹形状：'sine'（正弦）或 'chirp'（扫频），默认 'sine'
%     'amplitude'       - Part B轨迹幅值 rad，默认 0.5
%     'frequency'       - Part B轨迹频率 Hz（sine用），默认 0.1
%     'freq_start'      - Part B扫频起始频率 Hz，默认 0.05
%     'freq_end'        - Part B扫频结束频率 Hz，默认 0.5
%     'duration'        - Part B轨迹持续时间 s，默认 30
%     'output_file'     - 输出文件名，默认自动生成
%     'urdf_file'       - URDF文件路径，默认自动查找
%                         如果提供，Part A将自动从URDF读取关节限位设置angle_range
%
% 输出:
%   生成CSV轨迹文件，格式：time, l_leg_hip_yaw_joint, ..., r_leg_ankle_roll_joint
%   只有指定关节有运动，其他关节保持为0
%
% 示例:
%   % 生成Part A轨迹（正反转匀速）
%   generate_urdf_validation_trajectory('part_a', 'l_leg_hip_yaw_joint', ...
%       'velocity', 0.2, 'angle_range', 2*pi);
%
%   % 生成Part B轨迹（正弦）
%   generate_urdf_validation_trajectory('part_b', 'l_leg_hip_yaw_joint', ...
%       'trajectory_shape', 'sine', 'amplitude', 0.5, 'frequency', 0.1, 'duration', 30);

%% 参数解析
p = inputParser;
addParameter(p, 'Ts', 0.002, @isnumeric);
addParameter(p, 'velocity', 0.2, @isnumeric);
addParameter(p, 'angle_range', 2*pi, @isnumeric);
addParameter(p, 'trajectory_shape', 'sine', @(x) ismember(x, {'sine', 'chirp'}));
addParameter(p, 'amplitude', 0.5, @isnumeric);
addParameter(p, 'frequency', 0.1, @isnumeric);
addParameter(p, 'freq_start', 0.05, @isnumeric);
addParameter(p, 'freq_end', 0.5, @isnumeric);
addParameter(p, 'duration', 30, @isnumeric);
addParameter(p, 'output_file', '', @ischar);
addParameter(p, 'urdf_file', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

% 如果未指定urdf_file，尝试默认路径
if isempty(opts.urdf_file) && strcmp(trajectory_type, 'part_a')
    % 从当前文件位置向上查找URDF文件
    % URDF文件路径：noetix_description/urdf/E1.urdf
    current_dir = fileparts(mfilename('fullpath'));
    % 尝试多个可能的路径（从工作目录开始）
    possible_paths = {
        fullfile('noetix_description', 'urdf', 'E1.urdf');  % 从工作目录（matlab根）
        fullfile(current_dir, '..', 'noetix_description', 'urdf', 'E1.urdf');  % plan 在 matlab/plan
        fullfile(current_dir, '..', '..', '..', 'noetix_description', 'urdf', 'E1.urdf');
        fullfile(pwd, 'noetix_description', 'urdf', 'E1.urdf');
    };
    
    for i = 1:length(possible_paths)
        if exist(possible_paths{i}, 'file')
            opts.urdf_file = possible_paths{i};
            break;
        end
    end
end

%% 定义所有关节名称（按CSV格式顺序）
all_joint_names = {
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

% 定义关节名称映射：用户名称 -> URDF名称
joint_name_mapping = containers.Map({
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
}, {
    'leg_l1_joint';
    'leg_l2_joint';
    'leg_l3_joint';
    'leg_l4_joint';
    'leg_l5_joint';
    'leg_l6_joint';
    'leg_r1_joint';
    'leg_r2_joint';
    'leg_r3_joint';
    'leg_r4_joint';
    'leg_r5_joint';
    'leg_r6_joint'
});

% 查找目标关节索引
target_joint_idx = find(strcmp(all_joint_names, joint_name));
if isempty(target_joint_idx)
    error('未找到关节: %s。支持的关节名称请查看函数内部 all_joint_names', joint_name);
end

% 获取URDF中的关节名称
if isKey(joint_name_mapping, joint_name)
    urdf_joint_name = joint_name_mapping(joint_name);
else
    urdf_joint_name = '';
    warning('未找到关节 %s 的URDF映射，将使用默认angle_range', joint_name);
end

fprintf('===== 生成URDF验证测试轨迹 =====\n');
fprintf('轨迹类型: %s\n', trajectory_type);
fprintf('目标关节: %s (索引: %d)\n', joint_name, target_joint_idx);
fprintf('采样周期: %.4f s (%.0f Hz)\n', opts.Ts, 1/opts.Ts);

%% Part A: 从URDF读取关节限位（如果提供）
if strcmp(trajectory_type, 'part_a') && ~isempty(opts.urdf_file) && ~isempty(urdf_joint_name)
    % 尝试从URDF读取关节限位
    if exist(opts.urdf_file, 'file')
        try
            % 读取URDF文件
            xmlDoc = xmlread(opts.urdf_file);
            jointNodes = xmlDoc.getElementsByTagName('joint');
            numJoints = jointNodes.getLength();
            
            % 查找目标关节
            found_joint = false;
            for i = 0:numJoints - 1
                jointNode = jointNodes.item(i);
                jointNameAttr = char(jointNode.getAttribute('name'));
                
                if strcmp(jointNameAttr, urdf_joint_name)
                    % 找到目标关节，读取限位
                    limitElement = jointNode.getElementsByTagName('limit').item(0);
                    if ~isempty(limitElement)
                        lower = str2double(limitElement.getAttribute('lower'));
                        upper = str2double(limitElement.getAttribute('upper'));
                        
                        if ~isnan(lower) && ~isnan(upper)
                            % 使用URDF限位设置angle_range
                            % angle_range = upper - lower，但为了安全，留一些余量（90%）
                            opts.angle_range = (upper - lower) * 0.9;
                            fprintf('\n从URDF读取关节限位: %s\n', urdf_joint_name);
                            fprintf('  lower: %.4f rad, upper: %.4f rad\n', lower, upper);
                            fprintf('  使用角度范围: %.4f rad (%.0f%% 限位范围)\n', ...
                                opts.angle_range, 90);
                            found_joint = true;
                        end
                    end
                    break;
                end
            end
            
            if ~found_joint
                warning('在URDF中未找到关节 %s，使用默认angle_range', urdf_joint_name);
            end
        catch ME
            warning('读取URDF文件失败: %s，使用默认angle_range', ME.message);
        end
    else
        warning('URDF文件不存在: %s，使用默认angle_range', opts.urdf_file);
    end
end

%% 生成轨迹
if strcmp(trajectory_type, 'part_a')
    % ========== Part A: 正反转匀速轨迹 ==========
    fprintf('\n========== Part A: 正反转匀速轨迹 ==========\n');
    fprintf('匀速速度: %.3f rad/s\n', opts.velocity);
    fprintf('角度范围: %.3f rad (%.1f 圈)\n', opts.angle_range, opts.angle_range/(2*pi));
    
    % 计算每段所需时间
    t_forward = opts.angle_range / abs(opts.velocity);  % 正转时间
    t_return = t_forward;  % 回程时间（同样速度）
    t_reverse = t_forward;  % 反转时间
    
    % 生成时间序列
    % 段1：正转（从0到angle_range）
    n_forward = round(t_forward / opts.Ts);
    t_forward_vec = (0:(n_forward-1))' * opts.Ts;
    q_forward = opts.velocity * t_forward_vec;
    
    % 段2：回程（从angle_range回到0，不参与计算，但需要记录）
    n_return = round(t_return / opts.Ts);
    t_return_vec = t_forward_vec(end) + opts.Ts + (0:(n_return-1))' * opts.Ts;
    q_return = opts.angle_range - opts.velocity * (t_return_vec - t_return_vec(1));
    
    % 段3：反转（从0到-angle_range）
    n_reverse = round(t_reverse / opts.Ts);
    t_reverse_vec = t_return_vec(end) + opts.Ts + (0:(n_reverse-1))' * opts.Ts;
    q_reverse = -opts.velocity * (t_reverse_vec - t_reverse_vec(1));
    
    % 合并所有段
    t_all = [t_forward_vec; t_return_vec; t_reverse_vec];
    q_all = [q_forward; q_return; q_reverse];
    
    fprintf('正转段: %.2f s, %d 点\n', t_forward, n_forward);
    fprintf('回程段: %.2f s, %d 点（不参与计算）\n', t_return, n_return);
    fprintf('反转段: %.2f s, %d 点\n', t_reverse, n_reverse);
    fprintf('总时长: %.2f s\n', t_all(end));
    fprintf('总数据点数: %d\n', length(t_all));
    
elseif strcmp(trajectory_type, 'part_b')
    % ========== Part B: 带加速度轨迹 ==========
    fprintf('\n========== Part B: 带加速度轨迹 ==========\n');
    fprintf('轨迹形状: %s\n', opts.trajectory_shape);
    fprintf('持续时间: %.1f s\n', opts.duration);
    
    % 生成时间序列
    n_points = round(opts.duration / opts.Ts);
    t_all = (0:(n_points-1))' * opts.Ts;
    
    if strcmp(opts.trajectory_shape, 'sine')
        % 正弦轨迹：q = amplitude * sin(2*pi*frequency*t)
        fprintf('幅值: %.3f rad\n', opts.amplitude);
        fprintf('频率: %.3f Hz\n', opts.frequency);
        q_all = opts.amplitude * sin(2*pi*opts.frequency*t_all);
        
    elseif strcmp(opts.trajectory_shape, 'chirp')
        % 扫频轨迹：q = amplitude * sin(2*pi*phi(t))
        % phi(t) = freq_start*t + 0.5*(freq_end-freq_start)*t^2/duration
        fprintf('幅值: %.3f rad\n', opts.amplitude);
        fprintf('扫频范围: %.3f ~ %.3f Hz\n', opts.freq_start, opts.freq_end);
        phi = opts.freq_start*t_all + 0.5*(opts.freq_end-opts.freq_start)*t_all.^2/opts.duration;
        q_all = opts.amplitude * sin(2*pi*phi);
    end
    
    fprintf('总数据点数: %d\n', length(t_all));
    
else
    error('trajectory_type 必须是 ''part_a'' 或 ''part_b''');
end

%% 构建完整的关节轨迹矩阵（12个关节）
% 初始化：所有关节为0
n_total = length(t_all);
q_matrix = zeros(n_total, 12);

% 将目标关节的轨迹填入对应列
q_matrix(:, target_joint_idx) = q_all;

%% 生成输出文件名（默认写入 build/plan）
if isempty(opts.output_file)
    timestamp = datestr(now, 'yyyymmdd-HHMMSS');
    if strcmp(trajectory_type, 'part_a')
        fname = sprintf('%s_part_a_%s.csv', joint_name, timestamp);
    else
        fname = sprintf('%s_part_b_%s.csv', joint_name, timestamp);
    end
    opts.output_file = fullfile(get_build_dir('plan'), fname);
end

%% 保存为CSV格式
fprintf('\n保存轨迹文件...\n');
fid = fopen(opts.output_file, 'w');
if fid == -1
    error('无法创建文件: %s', opts.output_file);
end

% 写入表头
fprintf(fid, 'time');
for i = 1:length(all_joint_names)
    fprintf(fid, ',%s', all_joint_names{i});
end
fprintf(fid, '\n');

% 写入数据（时间保留5位小数，位置保留6位小数）
% 时间格式：0.00000, 0.00200, 0.00400, ...（对于Ts=0.002，500Hz）
for i = 1:n_total
    fprintf(fid, '%.5f', t_all(i));
    for j = 1:12
        fprintf(fid, ',%.6f', q_matrix(i, j));
    end
    fprintf(fid, '\n');
end

fclose(fid);

% 获取完整路径
[filepath, filename, ext] = fileparts(opts.output_file);
if isempty(filepath)
    full_path = fullfile(pwd, opts.output_file);
else
    full_path = opts.output_file;
end

fprintf('已保存轨迹文件: %s\n', opts.output_file);
fprintf('完整路径: %s\n', full_path);
fprintf('数据点数: %d\n', n_total);
fprintf('总时长: %.2f s\n', t_all(end));

%% 可视化预览
figure('Name', sprintf('URDF验证轨迹预览: %s - %s', trajectory_type, joint_name));
subplot(2,1,1);
plot(t_all, q_all, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel(sprintf('%s 位置 (rad)', joint_name));
if strcmp(trajectory_type, 'part_a')
    part_letter = 'A';
else
    part_letter = 'B';
end
title(sprintf('Part %s: %s 位置轨迹', part_letter, joint_name));

subplot(2,1,2);
qd_all = [0; diff(q_all)/opts.Ts];  % 数值微分得到速度
plot(t_all, qd_all, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel(sprintf('%s 速度 (rad/s)', joint_name));
if strcmp(trajectory_type, 'part_a')
    part_letter = 'A';
else
    part_letter = 'B';
end
title(sprintf('Part %s: %s 速度轨迹', part_letter, joint_name));

if strcmp(trajectory_type, 'part_b')
    % Part B还显示加速度
    figure('Name', sprintf('Part B加速度预览: %s', joint_name));
    qdd_all = [0; diff(qd_all)/opts.Ts];  % 数值微分得到加速度
    plot(t_all, qdd_all, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel(sprintf('%s 加速度 (rad/s²)', joint_name));
    title(sprintf('Part B: %s 加速度轨迹', joint_name));
end

fprintf('\n===== 轨迹生成完成 =====\n');

end
