function generate_multi_speed_trajectory(varargin)
% generate_multi_speed_trajectory  生成多速度档位往返匀速轨迹
%
% 功能：生成包含所有关节的多速度档位往返匀速轨迹
%       - 3个速度档位：最大速度的0.3、0.6、0.9倍
%       - 平滑轨迹规划（moveJ，5次多项式）
%       - 依次运动：左腿6个关节 -> 右腿6个关节
%       - 每个关节往返运动：lower -> upper -> lower
%       - 起始终止速度、加速度均为0，位置连续无跳变
%
% 输入参数（可选）:
%   varargin - 可选参数（名值对）:
%     'Ts'              - 采样周期，默认 0.002 s (500 Hz)
%     'urdf_file'       - URDF文件路径，默认自动查找 noetix_description/urdf/E1.urdf
%     'speed_ratios'    - 速度比例数组，默认 [0.3, 0.6, 0.9]
%     'accel_ratio'     - 加速度比例（相对于最大速度），默认 0.8
%                         用于自动计算加减速时间
%                         注意：accel_ratio 越大，加速度越大，越容易在有限距离内达到高速度
%                         如果速度达不到期望值，可以尝试增大 accel_ratio（例如 1.0 或 1.5）
%%     'output_file'     - 输出文件名，默认自动生成
%
% 输出:
%   生成CSV轨迹文件，格式：time, leg_l1_joint, leg_l2_joint, ..., leg_r6_joint
%
% 运动流程：
%   1. 每个关节从0开始，执行：0 -> upper -> lower -> upper -> 0（4段moveJ轨迹）
%   2. 每个速度档位都执行一次完整的往返运动
%   3. 依次处理所有关节（左腿6个关节 -> 右腿6个关节）
%   4. 所有段之间位置、速度、加速度连续（moveJ保证起始终止速度加速度为0）
%
% 碰撞检测：
%   - 当前腿运动时，该腿的roll_joint设为0
%   - 另一条腿运动时，该腿的roll_joint设为安全位置：
%     * 左腿roll_joint设置为最大（upper限位）
%     * 右腿roll_joint设置为最小（lower限位）
%   - 其余关节保持0位
%   - 当前运动的关节从0开始运动，同一条腿的其他关节保持0位置

%% 参数解析
p = inputParser;
addParameter(p, 'Ts', 0.005, @isnumeric);
addParameter(p, 'urdf_file', '', @ischar);
addParameter(p, 'speed_ratios', [0.3, 0.6, 0.9], @isnumeric);
addParameter(p, 'accel_ratio', 0.8, @isnumeric);  % 默认改为0.8（更大的加速度，有助于在有限距离内达到高速度）
addParameter(p, 'output_file', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

fprintf('===== 生成多速度档位往返匀速轨迹 =====\n');
fprintf('采样周期: %.4f s (%.0f Hz)\n', opts.Ts, 1/opts.Ts);
% 动态打印速度比例数组
if length(opts.speed_ratios) == 1
    fprintf('速度比例: %.4f\n', opts.speed_ratios(1));
elseif length(opts.speed_ratios) == 2
    fprintf('速度比例: [%.4f, %.4f]\n', opts.speed_ratios(1), opts.speed_ratios(2));
elseif length(opts.speed_ratios) >= 3
    fprintf('速度比例: [%.4f, %.4f, %.4f', opts.speed_ratios(1), opts.speed_ratios(2), opts.speed_ratios(3));
    if length(opts.speed_ratios) > 3
        for i = 4:length(opts.speed_ratios)
            fprintf(', %.4f', opts.speed_ratios(i));
        end
    end
    fprintf(']\n');
end
fprintf('加速度比例: %.2f（用于自动计算轨迹时间）\n', opts.accel_ratio);
fprintf('轨迹规划: moveJ（5次多项式，起始终止速度加速度为0）\n');

%% 查找URDF文件
if isempty(opts.urdf_file)
    current_dir = fileparts(mfilename('fullpath'));
    % URDF文件路径：noetix_description/urdf/E1.urdf
    possible_paths = {
        fullfile('noetix_description', 'urdf', 'E1.urdf');  % 从工作目录
        fullfile(current_dir, '..', '..', '..', 'noetix_description', 'urdf', 'E1.urdf');  % 从当前文件向上3级
        fullfile(current_dir, '..', '..', 'noetix_description', 'urdf', 'E1.urdf');  % 从当前文件向上2级
        fullfile(pwd, 'noetix_description', 'urdf', 'E1.urdf');  % 从当前工作目录
    };
    
    found_urdf = false;
    for i = 1:length(possible_paths)
        if exist(possible_paths{i}, 'file')
            opts.urdf_file = possible_paths{i};
            found_urdf = true;
            fprintf('找到URDF文件: %s\n', opts.urdf_file);
            break;
        end
    end
    
    if ~found_urdf
        error('未找到URDF文件 noetix_description/urdf/E1.urdf\n请确保文件存在或手动指定urdf_file参数');
    end
else
    if ~exist(opts.urdf_file, 'file')
        error('指定的URDF文件不存在: %s', opts.urdf_file);
    end
    fprintf('使用指定的URDF文件: %s\n', opts.urdf_file);
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

%% 从URDF读取关节限位和最大速度
fprintf('\n从URDF读取关节参数...\n');
xmlDoc = xmlread(opts.urdf_file);
jointNodes = xmlDoc.getElementsByTagName('joint');
numJoints = jointNodes.getLength();

joint_params = struct();
for i = 1:length(all_joint_names)
    joint_name = all_joint_names{i};
    if isKey(joint_name_mapping, joint_name)
        urdf_joint_name = joint_name_mapping(joint_name);
        
        % 在URDF中查找关节
        found = false;
        for j = 0:numJoints - 1
            jointNode = jointNodes.item(j);
            jointNameAttr = char(jointNode.getAttribute('name'));
            
            if strcmp(jointNameAttr, urdf_joint_name)
                limitElement = jointNode.getElementsByTagName('limit').item(0);
                if ~isempty(limitElement)
                    lower = str2double(limitElement.getAttribute('lower'));
                    upper = str2double(limitElement.getAttribute('upper'));
                    velocity = str2double(limitElement.getAttribute('velocity'));
                    
                    % 使用90%安全余量
                    joint_params.(joint_name).lower = lower * 0.9;
                    joint_params.(joint_name).upper = upper * 0.9;
                    joint_params.(joint_name).velocity_max = velocity;
                    found = true;
                    fprintf('  %s: 限位[%.4f, %.4f] rad, v_max=%.2f rad/s\n', ...
                        joint_name, joint_params.(joint_name).lower, ...
                        joint_params.(joint_name).upper, velocity);
                end
                break;
            end
        end
        
        if ~found
            warning('未找到关节 %s，使用默认值', urdf_joint_name);
            joint_params.(joint_name).lower = -pi;
            joint_params.(joint_name).upper = pi;
            joint_params.(joint_name).velocity_max = 5.0;
        end
    end
end

%% 生成三段式轨迹（加速+匀速+减速，最大化匀速段长度）
function [t_seg, q_seg] = generate_moveJ_trajectory(...
    q_start, q_end, v_max_used, accel_ratio, Ts, joint_lower, joint_upper)
    % 生成三段式轨迹：加速段（3次多项式平滑） + 匀速段（尽可能长） + 减速段（3次多项式平滑）
    % 边界条件：起始终止速度为0，匀速段速度 = v_max_used
    % 使用3次多项式可以更快达到目标速度，最大化匀速段长度
    % 
    % 优化策略：最小化加速/减速段距离，最大化匀速段长度
    % - 使用激进的加速策略（允许稍大的有效加速度），缩短加速/减速距离
    % - 将剩余的所有距离分配给匀速段，确保匀速时间尽可能长
    %
    % 输入:
    %   q_start, q_end - 起点、终点位置 (rad)
    %   v_max_used     - 匀速段速度 (rad/s)，方向由 q_end - q_start 决定
    %   accel_ratio    - 加速度比例，a_max = v_max_used * accel_ratio
    %   Ts             - 采样周期 (s)
    % 输出:
    %   t_seg - 相对时间向量 (从0开始)
    %   q_seg - 位置向量
    
    % 计算运动距离和方向
    dist = abs(q_end - q_start);
    direction = sign(q_end - q_start);
    if direction == 0, direction = 1; end
    
    if dist < 1e-6
        t_seg = [0; Ts];
        q_seg = [q_start; q_start];
        return;
    end
    
    % 加速度限制
    a_max = v_max_used * accel_ratio;
    
    % 为了最大化匀速段，使用最小化的加速/减速距离
    % 加速段最小距离：使用恒定加速度估算 dist_accel_min = v_max^2 / (2 * a_max)
    % 但为了尽可能长匀速段，我们使用更激进的加速策略
    dist_accel_min_theory = v_max_used^2 / (2 * a_max);
    
    % 使用更激进的加速策略：允许使用更大的有效加速度，缩短加速距离
    % 为了在有限距离内达到更高速度，使用更大的激进系数
    aggressive_factor = 2.0;  % 激进系数，允许更大的有效加速度（从1.2增加到2.0）
    dist_accel_min = dist_accel_min_theory / aggressive_factor;
    
    % 如果总距离太短，无法达到v_max_used，则只用加速+减速
    if dist_accel_min * 2 >= dist
        % 距离太短，无法达到v_max_used，使用实际能达到的最大速度
        % 使用恒定加速度模型：v_max^2 = 2 * a_max * dist_accel
        % 所以 v_actual = sqrt(2 * a_max * dist_accel)，其中 dist_accel = dist / 2
        dist_accel = dist / 2;
        dist_decel = dist / 2;
        dist_const = 0;
        v_actual = sqrt(2 * a_max * dist_accel);  % 实际能达到的最大速度（修正公式）
        
        % 如果距离严重不足，进一步增加有效加速度
        if v_actual < v_max_used * 0.7
            % 使用更大的有效加速度（允许峰值加速度超过a_max）
            effective_accel_factor = 2.0;  % 允许峰值加速度为 a_max 的2.0倍（更激进）
            v_actual = sqrt(2 * a_max * effective_accel_factor * dist_accel);
            v_actual = min(v_actual, v_max_used);  % 不超过期望速度
            % 更新 a_max 以反映更大的有效加速度
            a_max = a_max * effective_accel_factor;
        end
        
        % 调试信息（仅在距离不足时打印）
        if v_actual < v_max_used * 0.9
            % 静默处理，不打印警告（避免输出过多）
        end
    else
        % 最大化匀速段：使用最小化的加速/减速距离
        % 加速段和减速段各使用最小必要距离，剩余全部用于匀速段
        dist_accel = dist_accel_min;
        dist_decel = dist_accel_min;
        dist_const = dist - dist_accel - dist_decel;  % 剩余全部用于匀速段
        v_actual = v_max_used;  % 距离足够，应该能达到期望速度
        
        % 如果匀速段距离为负（理论上不应该发生），则使用最小距离
        if dist_const < 0
            dist_accel = dist / 2;
            dist_decel = dist / 2;
            dist_const = 0;
            % 使用修正的公式计算实际速度
            v_actual = sqrt(2 * a_max * dist_accel);
            % 如果仍然不足，允许更大的有效加速度
            if v_actual < v_max_used * 0.7
                effective_accel_factor = 1.5;
                v_actual = sqrt(2 * a_max * effective_accel_factor * dist_accel);
                v_actual = min(v_actual, v_max_used);
            end
        end
    end
    
    % 计算各段时间
    % 加速段：使用3次多项式从0加速到v_actual
    % 对于3次多项式，可以直接计算时间以确保末端速度达到v_actual
    % 3次多项式：qd(s) = a1 + 2*a2*s + 3*a3*s^2
    % 在s=1时：qd(1) = 2*a2 + 3*a3 = v_target_normalized / T_accel
    % 其中：a2 = 3*delta_q - v_target_normalized, a3 = v_target_normalized - 2*delta_q
    % 所以：qd(1) = 2*(3*delta_q - v_target_normalized) + 3*(v_target_normalized - 2*delta_q)
    %      = 6*delta_q - 2*v_target_normalized + 3*v_target_normalized - 6*delta_q
    %      = v_target_normalized
    % 因此：v_target_normalized / T_accel = v_actual * direction
    % 即：v_target_normalized = v_actual * direction * T_accel
    % 对于3次多项式，可以直接求解时间
    % 使用恒定加速度估算：T ≈ 2*dist_accel / v_actual（更保守）
    % 或使用加速度限制：T ≈ sqrt(3*dist_accel / a_max)（3次多项式的加速度特性）
    T_accel_est = 2 * dist_accel / v_actual;  % 基于平均速度的估算
    T_accel_est = max(T_accel_est, sqrt(3 * dist_accel / a_max));  % 加速度限制（3次多项式）
    T_accel_est = max(T_accel_est, 0.1);  % 最小时间
    
    % 迭代调整时间，确保末端速度达到v_actual（3次多项式也需要迭代以确保精确）
    T_accel = T_accel_est;
    max_iter = 10;
    for iter = 1:max_iter
        v_target_normalized_test = v_actual * direction * T_accel;
        a2_test = 3 * dist_accel - v_target_normalized_test;
        a3_test = v_target_normalized_test - 2 * dist_accel;
        % 计算s=1时的速度：qd(1) = 2*a2 + 3*a3
        qd_end_test = 2*a2_test + 3*a3_test;
        v_end_actual = abs(qd_end_test) / T_accel;
        
        if abs(v_end_actual - v_actual) / v_actual < 0.01
            break;  % 已达到目标速度
        end
        % 调整时间：如果速度偏小，缩短时间；如果速度偏大，延长时间
        T_accel = T_accel * v_end_actual / v_actual;
        T_accel = max(T_accel, 0.05);  % 确保不小于最小值
    end
    
    % 匀速段：使用所有剩余距离，尽可能长
    T_const = dist_const / v_actual;
    T_const = max(T_const, 0);  % 确保非负
    
    % 减速段：与加速段对称（使用相同的时间）
    T_decel = T_accel;
    
    % 生成加速段（3次多项式：从 q_start, qd=0 到 q_accel_end, qd=v_actual）
    % 边界条件：q(0)=q_start, qd(0)=0, q(1)=q_accel_end, qd(1)=v_actual*direction
    % 3次多项式更简单，可以更快达到目标速度，最大化匀速段
    q_accel_start = q_start;
    q_accel_end = q_start + direction * dist_accel;
    n_accel = max(round(T_accel / Ts), 5);
    s_accel = linspace(0, 1, n_accel)';
    delta_q_accel = q_accel_end - q_accel_start;
    v_target_normalized = v_actual * direction * T_accel;  % 归一化时间下的目标速度
    
    % 3次多项式系数（满足边界条件：qd(0)=0, qd(1)=v_target_normalized）
    % q(s) = a0 + a1*s + a2*s^2 + a3*s^3
    % 边界条件：
    %   q(0) = a0 = q_start
    %   qd(0) = a1 = 0
    %   q(1) = a0 + a2 + a3 = q_end => a2 + a3 = delta_q
    %   qd(1) = a1 + 2*a2 + 3*a3 = 2*a2 + 3*a3 = v_target_normalized
    % 解线性方程组：
    %   a2 + a3 = delta_q
    %   2*a2 + 3*a3 = v_target_normalized
    % 解得：
    %   a3 = v_target_normalized - 2*delta_q
    %   a2 = 3*delta_q - v_target_normalized
    a0_accel = q_accel_start;
    a1_accel = 0;
    a2_accel = 3 * delta_q_accel - v_target_normalized;
    a3_accel = v_target_normalized - 2 * delta_q_accel;
    
    q_accel = a0_accel + a1_accel*s_accel + a2_accel*s_accel.^2 + a3_accel*s_accel.^3;
    t_accel = s_accel * T_accel;
    
    % 生成匀速段（恒定速度）
    if dist_const > 1e-6
        n_const = max(round(T_const / Ts), 5);
        q_const_start = q_accel_end;
        t_const = t_accel(end) + Ts + (0:(n_const-1))' * Ts;
        q_const = q_const_start + direction * v_actual * (t_const - t_const(1));
    else
        t_const = [];
        q_const = [];
    end
    
    % 生成减速段（3次多项式：从 q_decel_start, qd=v_actual 到 q_end, qd=0）
    % 边界条件：q(0)=q_decel_start, qd(0)=v_actual*direction, q(1)=q_end, qd(1)=0
    % 3次多项式更简单，可以更快减速，最大化匀速段
    if ~isempty(q_const)
        q_decel_start = q_const(end);
    else
        q_decel_start = q_accel_end;
    end
    q_decel_end = q_end;
    n_decel = max(round(T_decel / Ts), 5);
    s_decel = linspace(0, 1, n_decel)';
    delta_q_decel = q_decel_end - q_decel_start;
    v_start_normalized = v_actual * direction * T_decel;  % 归一化时间下的起始速度
    
    % 3次多项式系数（满足边界条件：qd(0)=v_start_normalized, qd(1)=0）
    % q(s) = a0 + a1*s + a2*s^2 + a3*s^3
    % 边界条件：
    %   q(0) = a0 = q_decel_start
    %   qd(0) = a1 = v_start_normalized
    %   q(1) = a0 + a1 + a2 + a3 = q_end => a2 + a3 = delta_q - v_start_normalized
    %   qd(1) = a1 + 2*a2 + 3*a3 = v_start_normalized + 2*a2 + 3*a3 = 0 => 2*a2 + 3*a3 = -v_start_normalized
    % 解线性方程组：
    %   a2 + a3 = delta_q - v_start_normalized
    %   2*a2 + 3*a3 = -v_start_normalized
    % 解得：
    %   a3 = -v_start_normalized - 2*(delta_q - v_start_normalized) = -2*delta_q + v_start_normalized
    %   a2 = (delta_q - v_start_normalized) - a3 = 3*delta_q - 2*v_start_normalized
    a0_decel = q_decel_start;
    a1_decel = v_start_normalized;
    a2_decel = 3 * delta_q_decel - 2 * v_start_normalized;
    a3_decel = -2 * delta_q_decel + v_start_normalized;
    
    q_decel = a0_decel + a1_decel*s_decel + a2_decel*s_decel.^2 + a3_decel*s_decel.^3;
    
    if ~isempty(t_const)
        t_decel = t_const(end) + Ts + s_decel * T_decel;
    else
        t_decel = t_accel(end) + Ts + s_decel * T_decel;
    end
    
    % 合并三段轨迹
    t_seg = [t_accel; t_const; t_decel];
    q_seg = [q_accel; q_const; q_decel];
    
    % 确保边界条件精确满足
    q_seg(1) = q_start;
    q_seg(end) = q_end;
end

%% 生成完整轨迹
fprintf('\n生成轨迹...\n');
t_all = [];
q_matrix_all = zeros(0, 12);
t_current = 0;

% 找到第一个有效的关节，确定初始安全位置
first_joint_idx = [];
first_safe_positions = zeros(12, 1);
for joint_idx = 1:length(all_joint_names)
    joint_name = all_joint_names{joint_idx};
    if isfield(joint_params, joint_name)
        first_joint_idx = joint_idx;
        % 判断第一个关节属于哪条腿
        is_left_leg = startsWith(joint_name, 'l_leg_');
        is_right_leg = startsWith(joint_name, 'r_leg_');
        if is_left_leg
            % 左腿运动，右腿roll_joint设为最小（lower限位）
            if isfield(joint_params, 'r_leg_hip_roll_joint')
                first_safe_positions(8) = joint_params.r_leg_hip_roll_joint.lower;
            end
        elseif is_right_leg
            % 右腿运动，左腿roll_joint设为最大（upper限位）
            if isfield(joint_params, 'l_leg_hip_roll_joint')
                first_safe_positions(2) = joint_params.l_leg_hip_roll_joint.upper;
            end
        end
        break;
    end
end

% 添加开始轨迹：从全0位到第一个关节运动前的安全位置（使用moveJ）
if ~isempty(first_joint_idx)
    initial_positions = zeros(12, 1);
    % 找到需要变化的关节（roll_joint）
    changed_joints = find(abs(first_safe_positions - initial_positions) > 1e-6);
    
    if ~isempty(changed_joints)
        % 使用过渡速度生成轨迹
        transition_v_max = 0.5;  % 过渡速度 rad/s
        % 找到变化最大的关节来确定轨迹时间
        max_change = 0;
        for j = changed_joints'
            change = abs(first_safe_positions(j) - initial_positions(j));
            if change > max_change
                max_change = change;
            end
        end
        
        % 为变化最大的关节生成moveJ轨迹来确定时间
        [t_start_rel, ~] = generate_moveJ_trajectory(0, max_change, transition_v_max, opts.accel_ratio, opts.Ts);
        T_transition = t_start_rel(end);
        
        % 为每个变化的关节生成moveJ轨迹
        n_points = max(round(T_transition / opts.Ts), 10);
        t_start = linspace(0, T_transition, n_points)' + t_current;
        q_start = repmat(initial_positions', n_points, 1);
        
        for j = changed_joints'
            [~, q_joint] = generate_moveJ_trajectory(...
                initial_positions(j), first_safe_positions(j), transition_v_max, opts.accel_ratio, opts.Ts);
            % 插值到统一的时间点
            if length(q_joint) == n_points
                q_start(:, j) = q_joint;
            else
                % 重新采样
                t_joint_rel = linspace(0, T_transition, length(q_joint))';
                q_start(:, j) = interp1(t_joint_rel, q_joint, linspace(0, T_transition, n_points)', 'linear', 'extrap');
            end
        end
        
        t_all = [t_all; t_start];
        q_matrix_all = [q_matrix_all; q_start];
        t_current = t_start(end);
        fprintf('添加开始轨迹：从全0位到安全位置（%.2f秒，moveJ）\n', T_transition);
    end
end

% 依次处理每个关节（并记录第一档速度的块范围，用于对齐 meta）
meta_joint_block = zeros(12, 2);   % [start_idx, end_idx] 每关节第一档速度块
meta_q_act       = cell(12, 1);   % 每关节第一档速度块内的 q_act
prev_is_left_leg = false;  % 记录上一个关节是否属于左腿
for joint_idx = 1:length(all_joint_names)
    joint_name = all_joint_names{joint_idx};
    
    if ~isfield(joint_params, joint_name)
        continue;
    end
    
    fprintf('\n处理关节 %d/%d: %s\n', joint_idx, length(all_joint_names), joint_name);
    
    lower = joint_params.(joint_name).lower;
    upper = joint_params.(joint_name).upper;
    v_max = joint_params.(joint_name).velocity_max;
    
    fprintf('  限位: [%.4f, %.4f] rad\n', lower, upper);
    fprintf('  最大速度: %.2f rad/s\n', v_max);
    
    % 判断当前关节属于哪条腿
    is_left_leg = startsWith(joint_name, 'l_leg_');
    is_right_leg = startsWith(joint_name, 'r_leg_');
    
    % 检测左右腿切换：从左腿切换到右腿时，需要添加过渡轨迹
    if joint_idx > 1 && prev_is_left_leg && is_right_leg
        % 从左腿切换到右腿：需要调整roll_joint
        % 上一个状态：左腿roll_joint=0，右腿roll_joint=最小
        % 当前状态：右腿roll_joint=0，左腿roll_joint=最大
        prev_positions = q_matrix_all(end, :)';  % 上一个状态
        
        % 确定当前关节的安全位置
        next_safe_positions = zeros(12, 1);
        if isfield(joint_params, 'l_leg_hip_roll_joint')
            next_safe_positions(2) = joint_params.l_leg_hip_roll_joint.upper;  % 左腿roll_joint设为最大
        end
        % 右腿roll_joint保持0（当前腿运动）
        
        % 检查roll_joint是否需要变化
        changed_joints = [];
        if abs(prev_positions(2) - next_safe_positions(2)) > 1e-6  % 左腿roll_joint
            changed_joints = [changed_joints, 2];
        end
        if abs(prev_positions(8) - next_safe_positions(8)) > 1e-6  % 右腿roll_joint
            changed_joints = [changed_joints, 8];
        end
        
        if ~isempty(changed_joints)
            % 生成过渡轨迹
            transition_v_max = 0.5;  % 过渡速度 rad/s
            max_change = 0;
            for j = changed_joints
                change = abs(next_safe_positions(j) - prev_positions(j));
                if change > max_change
                    max_change = change;
                end
            end
            
            % 为变化最大的关节生成moveJ轨迹来确定时间
            [t_trans_rel, ~] = generate_moveJ_trajectory(0, max_change, transition_v_max, opts.accel_ratio, opts.Ts);
            T_transition = t_trans_rel(end);
            
            % 为每个变化的关节生成moveJ轨迹
            n_points = max(round(T_transition / opts.Ts), 10);
            t_trans = linspace(0, T_transition, n_points)' + t_current;
            q_trans = repmat(prev_positions', n_points, 1);
            
            for j = changed_joints
                [~, q_joint] = generate_moveJ_trajectory(...
                    prev_positions(j), next_safe_positions(j), transition_v_max, opts.accel_ratio, opts.Ts);
                % 插值到统一的时间点
                if length(q_joint) == n_points
                    q_trans(:, j) = q_joint;
                else
                    % 重新采样
                    t_joint_rel = linspace(0, T_transition, length(q_joint))';
                    q_trans(:, j) = interp1(t_joint_rel, q_joint, linspace(0, T_transition, n_points)', 'linear', 'extrap');
                end
            end
            
            t_all = [t_all; t_trans];
            q_matrix_all = [q_matrix_all; q_trans];
            t_current = t_trans(end);
            fprintf('  添加左右腿切换过渡轨迹：调整roll_joint位置（%.2f秒，moveJ）\n', T_transition);
        end
    end
    
    % 更新prev_is_left_leg
    prev_is_left_leg = is_left_leg;
    
    % 碰撞检测：确定安全位置
    % 1. 当前运动的关节从0开始（在轨迹中设置）
    % 2. 同一条腿的其他关节保持0（不运动）
    % 3. 当前腿运动时，该腿的roll_joint设为0
    % 4. 另一条腿运动时，该腿的roll_joint设为安全位置：左腿最大（upper），右腿最小（lower）
    % 5. 其余关节保持0位
    safe_positions = zeros(12, 1);  % 所有关节的默认安全位置（0）
    
    if is_left_leg
        % 左腿运动，左腿roll_joint设为0，右腿roll_joint设为最小（lower限位）
        if isfield(joint_params, 'r_leg_hip_roll_joint')
            safe_positions(8) = joint_params.r_leg_hip_roll_joint.lower;  % 右腿roll_joint设为最小
        end
        fprintf('  碰撞检测：左腿roll_joint设为0，右腿roll_joint设置为最小（lower限位），其余关节保持0位\n');
    elseif is_right_leg
        % 右腿运动，右腿roll_joint设为0，左腿roll_joint设为最大（upper限位）
        if isfield(joint_params, 'l_leg_hip_roll_joint')
            safe_positions(2) = joint_params.l_leg_hip_roll_joint.upper;  % 左腿roll_joint设为最大
        end
        fprintf('  碰撞检测：右腿roll_joint设为0，左腿roll_joint设置为最大（upper限位），其余关节保持0位\n');
    end
    
    % 注意：当前运动的关节位置会在轨迹生成时覆盖safe_positions
    
    % 第一档速度块起始行（用于导出对齐 meta）
    row_start_joint = size(q_matrix_all, 1) + 1;
    
    % 对每个速度比例生成轨迹
    for speed_idx = 1:length(opts.speed_ratios)
        speed_ratio = opts.speed_ratios(speed_idx);
        v_max_used = v_max * speed_ratio;
        
        fprintf('  速度档位 %d: %.1f%% (%.3f rad/s)\n', speed_idx, speed_ratio*100, v_max_used);
        
        % 轨迹序列：0 -> upper -> lower -> upper -> 0
        % 确保每个段之间位置、速度、加速度连续（moveJ保证起始终止速度加速度为0）
        
        % ========== 段1：0 -> upper（moveJ轨迹）==========
        [t_seg1_rel, q_seg1] = generate_moveJ_trajectory(...
            0, upper, v_max_used, opts.accel_ratio, opts.Ts);
        t_seg1 = t_seg1_rel + t_current;
        q_matrix_seg1 = repmat(safe_positions', length(q_seg1), 1);  % 设置所有关节为安全位置
        q_matrix_seg1(:, joint_idx) = q_seg1;  % 当前关节使用运动轨迹（从0开始）
        % 确保第一个点当前关节位置为0（数值精度）
        q_matrix_seg1(1, joint_idx) = 0;
        t_all = [t_all; t_seg1];
        q_matrix_all = [q_matrix_all; q_matrix_seg1];
        t_current = t_seg1(end);  % 连续连接，无间隔
        
        % ========== 段2：upper -> lower（moveJ轨迹）==========
        [t_seg2_rel, q_seg2] = generate_moveJ_trajectory(...
            upper, lower, v_max_used, opts.accel_ratio, opts.Ts);
        % 移除第一个点（与段1最后一个点重复），从第二个点开始
        t_seg2 = t_seg2_rel(2:end) + t_current;
        q_seg2 = q_seg2(2:end);
        q_matrix_seg2 = repmat(safe_positions', length(q_seg2), 1);  % 设置所有关节为安全位置
        q_matrix_seg2(:, joint_idx) = q_seg2;  % 当前关节使用运动轨迹
        t_all = [t_all; t_seg2];
        q_matrix_all = [q_matrix_all; q_matrix_seg2];
        t_current = t_seg2(end);  % 连续连接，无间隔
        
        % ========== 段3：lower -> upper（moveJ轨迹）==========
        [t_seg3_rel, q_seg3] = generate_moveJ_trajectory(...
            lower, upper, v_max_used, opts.accel_ratio, opts.Ts);
        % 移除第一个点（与段2最后一个点重复），从第二个点开始
        t_seg3 = t_seg3_rel(2:end) + t_current;
        q_seg3 = q_seg3(2:end);
        q_matrix_seg3 = repmat(safe_positions', length(q_seg3), 1);  % 设置所有关节为安全位置
        q_matrix_seg3(:, joint_idx) = q_seg3;  % 当前关节使用运动轨迹
        t_all = [t_all; t_seg3];
        q_matrix_all = [q_matrix_all; q_matrix_seg3];
        t_current = t_seg3(end);  % 连续连接，无间隔
        
        % ========== 段4：upper -> 0（moveJ轨迹）==========
        [t_seg4_rel, q_seg4] = generate_moveJ_trajectory(...
            upper, 0, v_max_used, opts.accel_ratio, opts.Ts);
        % 移除第一个点（与段3最后一个点重复），从第二个点开始
        t_seg4 = t_seg4_rel(2:end) + t_current;
        q_seg4 = q_seg4(2:end);
        q_matrix_seg4 = repmat(safe_positions', length(q_seg4), 1);  % 设置所有关节为安全位置
        q_matrix_seg4(:, joint_idx) = q_seg4;  % 当前关节使用运动轨迹
        t_all = [t_all; t_seg4];
        q_matrix_all = [q_matrix_all; q_matrix_seg4];
        t_current = t_seg4(end) + opts.Ts * 10;  % 间隔10个采样周期（准备下一个速度档位）
        
        % 第一档速度结束时记录该关节块范围与 q_act，供对齐精炼使用
        if speed_idx == 1
            row_end_joint = size(q_matrix_all, 1);
            meta_joint_block(joint_idx, 1) = row_start_joint;
            meta_joint_block(joint_idx, 2) = row_end_joint;
            meta_q_act{joint_idx} = q_matrix_all(row_start_joint:row_end_joint, joint_idx);
        end
    end
end

% 添加结束轨迹：从最后一个关节运动后的状态回到全0位（使用moveJ）
if ~isempty(q_matrix_all)
    final_positions = zeros(12, 1);
    last_positions = q_matrix_all(end, :)';  % 最后一个状态
    
    % 找到需要变化的关节
    changed_joints = find(abs(last_positions - final_positions) > 1e-6);
    
    if ~isempty(changed_joints)
        % 使用过渡速度生成轨迹
        transition_v_max = 0.5;  % 过渡速度 rad/s
        % 找到变化最大的关节来确定轨迹时间
        max_change = 0;
        for j = changed_joints'
            change = abs(final_positions(j) - last_positions(j));
            if change > max_change
                max_change = change;
            end
        end
        
        % 为变化最大的关节生成moveJ轨迹来确定时间
        [t_end_rel, ~] = generate_moveJ_trajectory(0, max_change, transition_v_max, opts.accel_ratio, opts.Ts);
        T_transition = t_end_rel(end);
        
        % 为每个变化的关节生成moveJ轨迹
        n_points = max(round(T_transition / opts.Ts), 10);
        t_end = linspace(0, T_transition, n_points)' + t_current;
        q_end = repmat(last_positions', n_points, 1);
        
        for j = changed_joints'
            [~, q_joint] = generate_moveJ_trajectory(...
                last_positions(j), final_positions(j), transition_v_max, opts.accel_ratio, opts.Ts);
            % 插值到统一的时间点
            if length(q_joint) == n_points
                q_end(:, j) = q_joint;
            else
                % 重新采样
                t_joint_rel = linspace(0, T_transition, length(q_joint))';
                q_end(:, j) = interp1(t_joint_rel, q_joint, linspace(0, T_transition, n_points)', 'linear', 'extrap');
            end
        end
        
        t_all = [t_all; t_end];
        q_matrix_all = [q_matrix_all; q_end];
        fprintf('添加结束轨迹：从当前状态回到全0位（%.2f秒，moveJ）\n', T_transition);
    end
end

fprintf('\n轨迹生成完成\n');
fprintf('总时长: %.2f s\n', t_all(end));
fprintf('总数据点数: %d\n', length(t_all));

%% 保存为CSV格式（默认写入 build/plan）
if isempty(opts.output_file)
    timestamp = datestr(now, 'yyyymmdd-HHMMSS');
    opts.output_file = fullfile(get_build_dir('plan'), sprintf('multi_speed_trajectory_%s.csv', timestamp));
end

fprintf('\n保存轨迹文件: %s\n', opts.output_file);
fid = fopen(opts.output_file, 'w');
if fid == -1
    error('无法创建文件: %s', opts.output_file);
end

% 写入表头（使用URDF关节名称）
fprintf(fid, 'time');
for i = 1:length(all_joint_names)
    joint_name = all_joint_names{i};
    if isKey(joint_name_mapping, joint_name)
        urdf_joint_name = joint_name_mapping(joint_name);
        fprintf(fid, ',%s', urdf_joint_name);
    else
        fprintf(fid, ',%s', joint_name);  % 如果找不到映射，使用原名称
    end
end
fprintf(fid, '\n');

% 写入数据
for i = 1:length(t_all)
    fprintf(fid, '%.5f', t_all(i));
    for j = 1:12
        fprintf(fid, ',%.6f', q_matrix_all(i, j));
    end
    fprintf(fid, '\n');
end

fclose(fid);

fprintf('已保存: %s\n', opts.output_file);
fprintf('完整路径: %s\n', fullfile(pwd, opts.output_file));

%% 保存对齐用 meta（与 CSV 同目录、同名_meta.mat）
[p_dir, n_base, ~] = fileparts(opts.output_file);
meta_file = fullfile(p_dir, [n_base '_meta.mat']);
forward_range_default = [0.10, 0.45];  % 与 compare_single_joint_torque_aligned 默认一致
Ts_meta = opts.Ts;
save(meta_file, 'meta_joint_block', 'meta_q_act', 'Ts_meta', 'forward_range_default', ...
    '-v7.3');
fprintf('已保存对齐 meta: %s\n', meta_file);

%% 计算速度和加速度
fprintf('\n计算速度和加速度...\n');
qd_matrix_all = zeros(size(q_matrix_all));  % 速度矩阵
qdd_matrix_all = zeros(size(q_matrix_all));  % 加速度矩阵

for joint_idx = 1:12
    q = q_matrix_all(:, joint_idx);
    
    % 计算速度：qd = diff(q) / diff(t)
    % 使用中心差分法：qd[i] = (q[i+1] - q[i]) / (t[i+1] - t[i])
    dt = diff(t_all);
    qd = [0; diff(q) ./ dt];  % 第一个点速度设为0，其他点用前向差分
    
    % 计算加速度：qdd = diff(qd) / diff(t)
    % 使用中心差分法：qdd[i] = (qd[i+1] - qd[i]) / (t[i+1] - t[i])
    qdd = [0; diff(qd) ./ dt];  % 第一个点加速度设为0，其他点用前向差分
    
    qd_matrix_all(:, joint_idx) = qd;
    qdd_matrix_all(:, joint_idx) = qdd;
end

%% 可视化预览：位置、速度、加速度
fprintf('绘制轨迹预览（位置、速度、加速度）...\n');
num_joints_to_plot = min(6, length(all_joint_names));  % 显示前6个关节
figure('Name', '多速度档位轨迹预览（位置、速度、加速度）', 'Position', [50, 50, 1400, 900]);

for joint_idx = 1:num_joints_to_plot
    joint_name = all_joint_names{joint_idx};
    
    % 子图1：位置
    subplot(num_joints_to_plot, 3, (joint_idx-1)*3 + 1);
    plot(t_all, q_matrix_all(:, joint_idx), 'b-', 'LineWidth', 1);
    grid on;
    xlabel('时间 (s)');
    ylabel('位置 (rad)');
    title(sprintf('%s - 位置', joint_name), 'FontSize', 10);
    % 添加限位线
    if isfield(joint_params, joint_name)
        hold on;
        yline(joint_params.(joint_name).lower, 'r--', 'LineWidth', 1);
        yline(joint_params.(joint_name).upper, 'r--', 'LineWidth', 1);
        hold off;
    end
    
    % 子图2：速度
    subplot(num_joints_to_plot, 3, (joint_idx-1)*3 + 2);
    plot(t_all, qd_matrix_all(:, joint_idx), 'g-', 'LineWidth', 1);
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (rad/s)');
    title(sprintf('%s - 速度', joint_name), 'FontSize', 10);
    % 添加速度限制线（如果有）
    if isfield(joint_params, joint_name)
        hold on;
        v_max = joint_params.(joint_name).velocity_max;
        yline(v_max, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('最大速度: %.2f', v_max));
        yline(-v_max, 'r--', 'LineWidth', 1);
        hold off;
    end
    
    % 子图3：加速度
    subplot(num_joints_to_plot, 3, (joint_idx-1)*3 + 3);
    plot(t_all, qdd_matrix_all(:, joint_idx), 'r-', 'LineWidth', 1);
    grid on;
    xlabel('时间 (s)');
    ylabel('加速度 (rad/s²)');
    title(sprintf('%s - 加速度', joint_name), 'FontSize', 10);
    
    % 显示统计信息
    v_max_actual = max(abs(qd_matrix_all(:, joint_idx)));
    a_max_actual = max(abs(qdd_matrix_all(:, joint_idx)));
    if isfield(joint_params, joint_name)
        v_max_limit = joint_params.(joint_name).velocity_max;
        fprintf('  %s: 最大速度=%.3f rad/s (限制=%.2f), 最大加速度=%.3f rad/s²\n', ...
            joint_name, v_max_actual, v_max_limit, a_max_actual);
    else
        fprintf('  %s: 最大速度=%.3f rad/s, 最大加速度=%.3f rad/s²\n', ...
            joint_name, v_max_actual, a_max_actual);
    end
end

sgtitle('多速度档位轨迹预览（位置、速度、加速度）', 'FontSize', 12, 'FontWeight', 'bold');

fprintf('\n===== 完成 =====\n');

end
