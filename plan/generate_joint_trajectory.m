function varargout = generate_joint_trajectory(trajectory_type, varargin)
% generate_joint_trajectory  生成单关节测试轨迹（用于真机测试）
%
% 功能：生成多档匀速轨迹或多档恒定加速度轨迹，保存为文件供真机测试使用
%       使用MOVEJ（5次多项式）确保速度和轨迹变化丝滑
%
% 输入:
%   trajectory_type - 轨迹类型：
%                     'velocity'  : 多档匀速轨迹（用于摩擦辨识）
%                     'acceleration': 多档恒定加速度轨迹（用于I_a辨识）
%   varargin - 可选参数（名值对）:
%     'Ts'              - 采样周期，默认 0.01 s (100 Hz)
%     'v_max'           - 关节最大速度 rad/s，默认 2.0
%     'a_max'           - 关节最大加速度 rad/s²，默认 10.0
%     'output_file'     - 输出文件名，默认 'joint_trajectory.csv'
%     'output_format'   - 输出格式：'csv' 或 'mat'，默认 'csv'
%     'accel_ratio'     - 加速度比例（相对于最大速度），默认 0.5
%                         用于MOVEJ轨迹规划
%
% 输出:
%   生成轨迹文件，包含列：时间, 位置, 速度, 加速度
%
% 示例:
%   % 生成多档匀速轨迹
%   generate_joint_trajectory('velocity', 'v_max', 4.0, 'output_file', 'vel_traj.csv');
%
%   % 生成多档恒定加速度轨迹
%   generate_joint_trajectory('acceleration', 'a_max', 15.0, 'output_file', 'accel_traj.csv');

%% 参数解析
p = inputParser;
addParameter(p, 'Ts', 0.01, @isnumeric);
addParameter(p, 'v_max', 2.0, @isnumeric);
addParameter(p, 'a_max', 10.0, @isnumeric);
addParameter(p, 'output_file', 'joint_trajectory.csv', @ischar);
addParameter(p, 'output_format', 'csv', @(x) ismember(x, {'csv', 'mat'}));
addParameter(p, 'accel_ratio', 0.5, @isnumeric);  % 加速度比例，用于MOVEJ规划
addParameter(p, 'do_plot', true, @islogical);
addParameter(p, 'q_limit', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));  % 关节限位 ±q_limit (rad)，空表示不限制
parse(p, varargin{:});
opts = p.Results;
if isempty(opts.q_limit)
    opts.q_limit = inf;
end
% 默认输出到 build/plan
[out_dir, ~, ~] = fileparts(opts.output_file);
if isempty(out_dir)
    opts.output_file = fullfile(get_build_dir('plan'), opts.output_file);
end

fprintf('===== 生成关节测试轨迹（使用MOVEJ） =====\n');
fprintf('轨迹类型: %s\n', trajectory_type);
fprintf('采样周期: %.3f s\n', opts.Ts);
fprintf('最大速度: %.2f rad/s\n', opts.v_max);
fprintf('最大加速度: %.2f rad/s²\n', opts.a_max);
fprintf('加速度比例: %.2f\n', opts.accel_ratio);
if isfinite(opts.q_limit)
    fprintf('关节限位: ±%.2f rad\n', opts.q_limit);
end

%% 生成轨迹
if strcmp(trajectory_type, 'velocity')
    % ========== 多档匀速轨迹（用于摩擦辨识）- 最简方案 ==========
    % 速度档位正负交替（+0.01,-0.01,+0.02,-0.02,...），无中间0速档；每档段间回到位置0，MOVEJ 加速→匀速→减速
    fprintf('\n生成多档匀速轨迹（正负交替，每档回到0，无零速档）...\n');
    
    v_ultra_low = 0.01:0.01:0.10;
    v_ultra_low = repelem(v_ultra_low, 2);
    v_ultra_low(2:2:end) = -v_ultra_low(2:2:end);
    v_low = 0.15:0.05:0.30;
    v_low = repelem(v_low, 2);
    v_low(2:2:end) = -v_low(2:2:end);
    v_mid = 0.4:0.1:0.9;
    v_mid = repelem(v_mid, 2);
    v_mid(2:2:end) = -v_mid(2:2:end);
    if opts.v_max > 1.0
        n_high = 10;
        v_high = linspace(1.0, opts.v_max, n_high);
    else
        n_high = 1;
        v_high = opts.v_max;
    end
    v_high = repelem(v_high, 2);
    v_high(2:2:end) = -v_high(2:2:end);
    speed_list = [v_ultra_low, v_low, v_mid, v_high];
    
    ultra_low_speeds = [0.01, 0.02, 0.05, 0.08, 0.10];
    ultra_low_durations = [40, 40, 35, 35, 30];
    low_speeds = [0.15, 0.20, 0.25, 0.30];
    low_durations = [15, 15, 12, 12];
    v_mid_abs = 0.4:0.1:0.9;
    duration_mid = 5.0 * 2 * pi ./ v_mid_abs + 2.0;
    v_high_abs = linspace(1.0, opts.v_max, n_high);
    duration_high = max(15.0 * 2 * pi ./ v_high_abs + 5.0, 20.0);
    
    fprintf('速度档位: %d 档（正负交替），无零速档\n', length(speed_list));
    
    t_all = [];
    q_all = [];
    qd_all = [];
    qdd_all = [];
    t_curr = 0;
    q_curr = 0;
    qd_curr = 0;
    
    for k = 1:length(speed_list)
        v = speed_list(k);
        v_abs = abs(v);
        
        % 段间：从当前位置/速度用 MOVEJ 回到位置 0
        if abs(q_curr) > 0.001 || abs(qd_curr) > 0.001
            [t_decel_rel, q_decel, qd_decel, qdd_decel] = generate_moveJ_trajectory(...
                q_curr, 0, max(v_abs, abs(qd_curr)), opts.accel_ratio, opts.Ts);
            t_decel = t_curr + t_decel_rel;
            if ~isempty(t_all) && abs(t_decel(1) - t_curr) < opts.Ts * 0.5
                t_decel = t_decel(2:end); q_decel = q_decel(2:end);
                qd_decel = qd_decel(2:end); qdd_decel = qdd_decel(2:end);
            end
            if ~isempty(t_decel)
                t_all = [t_all; t_decel]; q_all = [q_all; q_decel];
                qd_all = [qd_all; qd_decel]; qdd_all = [qdd_all; qdd_decel];
                t_curr = t_decel(end); q_curr = 0; qd_curr = 0;
            end
        end
        
        % 本档：从 (0,0) MOVEJ 加速 → 匀速 → MOVEJ 减速到 (0,0)
        if v_abs <= 0.10
            idx = find(abs(ultra_low_speeds - v_abs) < 0.001);
            if ~isempty(idx), duration = ultra_low_durations(idx(1)); else, duration = interp1(ultra_low_speeds, ultra_low_durations, v_abs, 'linear', 'extrap'); end
        elseif v_abs <= 0.30
            idx = find(abs(low_speeds - v_abs) < 0.001);
            if ~isempty(idx), duration = low_durations(idx(1)); else, duration = interp1(low_speeds, low_durations, v_abs, 'linear', 'extrap'); end
        elseif v_abs <= 0.9
            idx_mid = find(abs(v_mid_abs - v_abs) < 0.05);
            if ~isempty(idx_mid), duration = duration_mid(idx_mid(1)); else, duration = 3.0 * 2 * pi / v_abs + 1.0; end
        else
            idx_high = find(abs(v_high_abs - v_abs) < 0.1);
            if ~isempty(idx_high), duration = duration_high(idx_high(1)); else, duration = max(15.0 * 2 * pi / v_abs + 5.0, 20.0); end
        end
        
        a_max_used = v_abs * opts.accel_ratio;
        dist_accel = v_abs^2 / (3 * a_max_used);
        dist_decel = dist_accel;
        T_accel_est = sqrt(4 * dist_accel / a_max_used);
        duration_const = max(duration - 2*T_accel_est, duration * 0.7);
        dist_const = v_abs * duration_const;
        
        % 加速段：0 → q_accel_end（MOVEJ）
        q_accel_end = sign(v) * dist_accel;
        [t_accel_rel, q_accel, qd_accel, qdd_accel] = generate_moveJ_trajectory(0, q_accel_end, v_abs, opts.accel_ratio, opts.Ts);
        t_accel = t_curr + t_accel_rel;
        if ~isempty(t_all) && abs(t_accel(1) - t_curr) < opts.Ts * 0.5
            t_accel = t_accel(2:end); q_accel = q_accel(2:end); qd_accel = qd_accel(2:end); qdd_accel = qdd_accel(2:end);
        end
        t_all = [t_all; t_accel]; q_all = [q_all; q_accel]; qd_all = [qd_all; qd_accel]; qdd_all = [qdd_all; qdd_accel];
        t_curr = t_accel(end); q_curr = q_accel(end); qd_curr = qd_accel(end);
        
        % 匀速段
        n_points_const = round(duration_const / opts.Ts);
        if n_points_const > 0
            q_const_start = q_curr;
            t_const = t_curr + (0:(n_points_const-1))' * opts.Ts;
            t_rel_const = (0:(n_points_const-1))' * opts.Ts;
            q_const = q_const_start + v * t_rel_const;
            qd_const = v * ones(n_points_const, 1);
            qdd_const = zeros(n_points_const, 1);
            t_all = [t_all; t_const]; q_all = [q_all; q_const]; qd_all = [qd_all; qd_const]; qdd_all = [qdd_all; qdd_const];
            t_curr = t_const(end); q_curr = q_const(end); qd_curr = v;
        end
        
        % 减速段：q_curr → 0（MOVEJ）
        [t_decel_rel, q_decel, qd_decel, qdd_decel] = generate_moveJ_trajectory(q_curr, 0, v_abs, opts.accel_ratio, opts.Ts);
        t_decel = t_curr + t_decel_rel;
        if ~isempty(t_all) && abs(t_decel(1) - t_curr) < opts.Ts * 0.5
            t_decel = t_decel(2:end); q_decel = q_decel(2:end); qd_decel = qd_decel(2:end); qdd_decel = qdd_decel(2:end);
        end
        if ~isempty(t_decel)
            t_all = [t_all; t_decel]; q_all = [q_all; q_decel]; qd_all = [qd_all; qd_decel]; qdd_all = [qdd_all; qdd_decel];
            t_curr = t_decel(end); q_curr = 0; qd_curr = 0;
        end
    end
    
    fprintf('总时长: %.2f s\n', t_all(end));
    fprintf('总数据点数: %d\n', length(t_all));
    
elseif strcmp(trajectory_type, 'acceleration')
    % ========== 多档恒定加速度轨迹（用于I_a辨识）==========
    fprintf('\n生成多档恒定加速度轨迹...\n');
    
    % 加速度档位设置
    % 小加速度：±0.1～±1.0，步长 0.1
    a_small = 0.1:0.1:1.0;
    a_small_all = [a_small, -a_small];
    
    % 中加速度：±1.5～±5.0，步长 0.5
    a_mid = 1.5:0.5:5.0;
    a_mid_all = [a_mid, -a_mid];
    
    % 大加速度：±6.0～±a_max，步长 1.0
    a_large = 6.0:1.0:opts.a_max;
    a_large_all = [a_large, -a_large];
    
    % 合并所有加速度档位
    accel_list = [a_small_all, a_mid_all, a_large_all];
    
    % 每个加速度档的持续时间（大幅增加以获得更多数据用于I_a辨识）
    duration_per_accel = 10.0;  % 每个加速度保持 10 秒（增加100%）
    transition_time = 0.5;      % 过渡时间（可选，已不使用）
    
    fprintf('加速度档位数: %d\n', length(accel_list));
    fprintf('每档持续时间: %.1f s\n', duration_per_accel);
    fprintf('加速度范围: %.2f ～ %.2f rad/s²\n', min(accel_list), max(accel_list));
    
    % 生成轨迹（使用MOVEJ）
    t_all = [];
    q_all = [];
    qd_all = [];
    qdd_all = [];
    t_curr = 0;
    q_curr = 0;  % 当前位置（用于平滑过渡）
    qd_curr = 0; % 当前速度（用于平滑过渡）
    
    for k = 1:length(accel_list)
        a_const = accel_list(k);
        
        % 如果当前位置或速度不是0，需要先减速回到0（使用MOVEJ平滑过渡）
        if abs(q_curr) > 0.001 || abs(qd_curr) > 0.001
            % 使用MOVEJ从当前位置减速回到0
            v_max_used = max(abs(qd_curr), 0.1);  % 至少0.1 rad/s
            [t_decel_rel, q_decel, qd_decel, qdd_decel] = generate_moveJ_trajectory(...
                q_curr, 0, v_max_used, opts.accel_ratio, opts.Ts);
            t_decel = t_curr + t_decel_rel;
            
            % 移除第一个点（如果与上一段重复）
            if ~isempty(t_all) && abs(t_decel(1) - t_curr) < opts.Ts * 0.5
                t_decel = t_decel(2:end);
                q_decel = q_decel(2:end);
                qd_decel = qd_decel(2:end);
                qdd_decel = qdd_decel(2:end);
            end
            
            % 合并减速段
            if ~isempty(t_decel)
                t_all = [t_all; t_decel];
                q_all = [q_all; q_decel];
                qd_all = [qd_all; qd_decel];
                qdd_all = [qdd_all; qdd_decel];
                t_curr = t_decel(end);
                q_curr = 0;
                qd_curr = 0;
            end
        end
        
        % 计算恒定加速度段的目标速度和距离
        % 目标速度：v_target = a_const * duration_per_accel，但不超过v_max
        v_target = sign(a_const) * min(abs(a_const) * duration_per_accel, opts.v_max);
        
        % 如果目标速度太小，跳过这个加速度档
        if abs(v_target) < 0.01
            continue;
        end
        
        % 估算恒定加速度段的距离和时间
        % 从v=0加速到v_target需要的时间：t_accel = v_target / a_const
        % 但为了保持恒定加速度，我们需要分段处理
        % 简化方案：使用MOVEJ加速到中间位置，然后保持恒定加速度，最后MOVEJ减速
        
        % 1. 加速段：使用MOVEJ从0加速到q_accel_end
        % 估算加速距离：假设加速到v_target，距离约为 v_target^2 / (2*a_const)
        % 但MOVEJ会平滑加速，我们使用一个合理的估算
        v_max_used = abs(v_target);
        dist_accel = v_target^2 / (2 * abs(a_const));  % 估算加速距离
        q_accel_end = sign(a_const) * dist_accel;
        
        [t_accel_rel, q_accel, qd_accel, qdd_accel] = generate_moveJ_trajectory(...
            0, q_accel_end, v_max_used, opts.accel_ratio, opts.Ts);
        t_accel = t_curr + t_accel_rel;
        
        % 移除第一个点（如果与上一段重复）
        if ~isempty(t_all) && abs(t_accel(1) - t_curr) < opts.Ts * 0.5
            t_accel = t_accel(2:end);
            q_accel = q_accel(2:end);
            qd_accel = qd_accel(2:end);
            qdd_accel = qdd_accel(2:end);
        end
        
        % 合并加速段
        if ~isempty(t_accel)
            t_all = [t_all; t_accel];
            q_all = [q_all; q_accel];
            qd_all = [qd_all; qd_accel];
            qdd_all = [qdd_all; qdd_accel];
            t_curr = t_accel(end);
            q_curr = q_accel(end);
            qd_curr = qd_accel(end);
        end
        
        % 2. 恒定加速度段：从当前状态开始，保持恒定加速度a_const
        % 计算恒定加速度段的持续时间
        % 总时间duration_per_accel，减去加速和减速时间
        T_accel_est = t_accel_rel(end);
        T_decel_est = T_accel_est;  % 假设减速时间与加速时间相同
        duration_const = max(duration_per_accel - T_accel_est - T_decel_est, duration_per_accel * 0.3);
        
        % 恒定加速度段：从q_curr, qd_curr开始
        n_points_const = round(duration_const / opts.Ts);
        if n_points_const > 0
            t_const = t_curr + (0:(n_points_const-1))' * opts.Ts;
            t_rel_const = (0:(n_points_const-1))' * opts.Ts;
            
            % 恒定加速度：qdd = a_const
            qdd_const = a_const * ones(n_points_const, 1);
            
            % 速度：qd = qd_curr + a_const * t_rel
            qd_const = qd_curr + a_const * t_rel_const;
            
            % 检查速度限制
            if any(abs(qd_const) > opts.v_max)
                % 截断到速度限制
                idx_valid = abs(qd_const) <= opts.v_max;
                if any(idx_valid)
                    qdd_const = qdd_const(idx_valid);
                    qd_const = qd_const(idx_valid);
                    t_const = t_const(idx_valid);
                    t_rel_const = t_rel_const(idx_valid);
                    n_points_const = length(t_const);
                else
                    n_points_const = 0;
                end
            end
            
            if n_points_const > 0
                % 位置：q = q_curr + qd_curr*t_rel + 0.5*a_const*t_rel²
                q_const = q_curr + qd_curr * t_rel_const + 0.5 * a_const * t_rel_const.^2;
                
                % 合并恒定加速度段
                t_all = [t_all; t_const];
                q_all = [q_all; q_const];
                qd_all = [qd_all; qd_const];
                qdd_all = [qdd_all; qdd_const];
                t_curr = t_const(end);
                q_curr = q_const(end);
                qd_curr = qd_const(end);
            end
        end
        
        % 3. 减速段：使用MOVEJ从当前位置减速到0
        if abs(qd_curr) > 0.01
            v_max_used = abs(qd_curr);
            [t_decel_rel, q_decel, qd_decel, qdd_decel] = generate_moveJ_trajectory(...
                q_curr, 0, v_max_used, opts.accel_ratio, opts.Ts);
            t_decel = t_curr + t_decel_rel;
            
            % 移除第一个点（如果与上一段重复）
            if ~isempty(t_all) && abs(t_decel(1) - t_curr) < opts.Ts * 0.5
                t_decel = t_decel(2:end);
                q_decel = q_decel(2:end);
                qd_decel = qd_decel(2:end);
                qdd_decel = qdd_decel(2:end);
            end
            
            % 合并减速段
            if ~isempty(t_decel)
                t_all = [t_all; t_decel];
                q_all = [q_all; q_decel];
                qd_all = [qd_all; qd_decel];
                qdd_all = [qdd_all; qdd_decel];
                t_curr = t_decel(end);
                q_curr = 0;
                qd_curr = 0;
            end
        end
    end
    
    fprintf('总时长: %.2f s\n', t_all(end));
    fprintf('总数据点数: %d\n', length(t_all));
    
else
    error('trajectory_type 必须是 ''velocity'' 或 ''acceleration''');
end

%% 构建 trajectory 结构体（用于返回或保存）
trajectory.time = t_all;
trajectory.position = q_all;
trajectory.velocity = qd_all;
trajectory.acceleration = qdd_all;
trajectory.type = trajectory_type;
trajectory.Ts = opts.Ts;
trajectory.v_max = opts.v_max;
trajectory.a_max = opts.a_max;

%% 保存轨迹文件（仅当需要写入文件时）
fprintf('\n保存轨迹文件...\n');

if strcmp(opts.output_format, 'csv')
    % 保存为 CSV 格式：时间, 位置, 速度, 加速度
    trajectory_data = [t_all, q_all];
    
    % 先写入表头，再写入数据
    fid = fopen(opts.output_file, 'w');
    fprintf(fid, 'time,position\n');
    fclose(fid);
    
    % 追加数据
    writematrix(trajectory_data, opts.output_file, 'WriteMode', 'append', 'Delimiter', ',');
    
    fprintf('已保存: %s\n', opts.output_file);
    fprintf('格式: CSV (时间, 位置, 速度, 加速度)\n');
    
elseif strcmp(opts.output_format, 'mat')
    % 保存为 MAT 格式（若调用方需要返回值则不写文件，只生成 CSV 时用）
    if nargout < 1 && ~isempty(opts.output_file)
        save(opts.output_file, 'trajectory');
        fprintf('已保存: %s\n', opts.output_file);
    end
    fprintf('格式: MAT (结构体 trajectory)\n');
end

%% 计算统计信息
v_max_actual = max(abs(qd_all));
a_max_actual = max(abs(qdd_all));
q_max = max(q_all);
q_min = min(q_all);

fprintf('\n轨迹统计信息:\n');
fprintf('  位置范围: [%.4f, %.4f] rad\n', q_min, q_max);
fprintf('  最大速度: %.4f rad/s (限制: %.2f rad/s)\n', v_max_actual, opts.v_max);
fprintf('  最大加速度: %.4f rad/s² (限制: %.2f rad/s²)\n', a_max_actual, opts.a_max);

%% 可视化预览（可选）
if opts.do_plot
    figure('Name', sprintf('单关节轨迹预览: %s', trajectory_type), ...
           'Position', [100, 100, 1200, 800]);
    
    % 子图1：位置
    subplot(3, 1, 1);
    plot(t_all, q_all, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)', 'FontSize', 11);
    ylabel('位置 q (rad)', 'FontSize', 11);
    title(sprintf('位置轨迹 (范围: [%.3f, %.3f] rad)', q_min, q_max), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    % 子图2：速度
    subplot(3, 1, 2);
    plot(t_all, qd_all, 'g-', 'LineWidth', 1.5);
    grid on;
    hold on;
    % 添加速度限制线
    yline(opts.v_max, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('最大速度限制: %.2f', opts.v_max));
    yline(-opts.v_max, 'r--', 'LineWidth', 1.5);
    hold off;
    xlabel('时间 (s)', 'FontSize', 11);
    ylabel('速度 q̇ (rad/s)', 'FontSize', 11);
    title(sprintf('速度轨迹 (最大: %.4f rad/s)', v_max_actual), ...
          'FontSize', 12, 'FontWeight', 'bold');
    % 设置y轴范围
    if max(qd_all) - min(qd_all) > 1e-6
        ylim([min(qd_all)*1.1, max(qd_all)*1.1]);
    else
        ylim([-0.1, 0.1]);
    end
    legend('速度', '速度限制', 'Location', 'best', 'FontSize', 9);
    
    % 子图3：加速度
    subplot(3, 1, 3);
    plot(t_all, qdd_all, 'r-', 'LineWidth', 1.5);
    grid on;
    hold on;
    % 添加加速度限制线
    yline(opts.a_max, 'm--', 'LineWidth', 1.5, 'DisplayName', sprintf('最大加速度限制: %.2f', opts.a_max));
    yline(-opts.a_max, 'm--', 'LineWidth', 1.5);
    hold off;
    xlabel('时间 (s)', 'FontSize', 11);
    ylabel('加速度 q̈ (rad/s²)', 'FontSize', 11);
    title(sprintf('加速度轨迹 (最大: %.4f rad/s²)', a_max_actual), ...
          'FontSize', 12, 'FontWeight', 'bold');
    % 检查数据范围是否有效（避免全为0时ylim报错）
    if max(qdd_all) - min(qdd_all) > 1e-6
        ylim([min(qdd_all)*1.1, max(qdd_all)*1.1]);
    else
        ylim([-0.1, 0.1]);  % 匀速轨迹加速度为0，设置固定范围
    end
    legend('加速度', '加速度限制', 'Location', 'best', 'FontSize', 9);
    
    % 添加总标题
    sgtitle(sprintf('单关节轨迹预览 (%s类型)', trajectory_type), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

fprintf('\n===== 轨迹生成完成 =====\n');
fprintf('文件: %s\n', opts.output_file);
fprintf('数据点数: %d\n', length(t_all));
fprintf('总时长: %.2f s\n', t_all(end));

if nargout >= 1
    varargout{1} = trajectory;
end

end

%% MOVEJ轨迹生成函数（加速 → 匀速 → 减速）- 局部函数
function [t_seg, q_seg, qd_seg, qdd_seg] = generate_moveJ_trajectory(...
    q_start, q_end, v_max_used, accel_ratio, Ts)
    % 生成 moveJ 轨迹：加速段(5次多项式) → 匀速段 → 减速段(5次多项式)
    % 速度、加速度在段间连续；若总距离过短则退化为单段 5 次多项式（无匀速段）。
    
    dist = abs(q_end - q_start);
    sign_dir = sign(q_end - q_start);
    
    if dist < 1e-6
        t_seg = [0; Ts];
        q_seg = [q_start; q_start];
        qd_seg = [0; 0];
        qdd_seg = [0; 0];
        return;
    end
    
    a_max = v_max_used * accel_ratio;
    % 加减速段：0↔v_max 用 5 次多项式，时间 T_accel，距离 d_accel = v_max^2/(2*a_max)
    T_accel = v_max_used / a_max;
    d_accel = v_max_used^2 / (2 * a_max);
    T_accel = max(T_accel, 0.25);  % 避免过短
    
    % 距离不足以出现匀速段时，退化为单段 5 次多项式（原逻辑）
    if dist < 2 * d_accel
        T_from_vel = 15/8 * dist / v_max_used;
        T_from_accel = sqrt(6 * dist / a_max);
        T = max(max(T_from_vel, T_from_accel), 0.5);
        n_points = max(round(T / Ts), 10);
        s = linspace(0, 1, n_points)';
        delta_q = q_end - q_start;
        a0 = q_start; a1 = 0; a2 = 0;
        a3 = 10 * delta_q; a4 = -15 * delta_q; a5 = 6 * delta_q;
        t_seg = s * T;
        q_seg = a0 + a1*s + a2*s.^2 + a3*s.^3 + a4*s.^4 + a5*s.^5;
        dq_ds = a1 + 2*a2*s + 3*a3*s.^2 + 4*a4*s.^3 + 5*a5*s.^4;
        qd_seg = dq_ds / T;
        d2q_ds2 = 2*a2 + 6*a3*s + 12*a4*s.^2 + 20*a5*s.^3;
        qdd_seg = d2q_ds2 / (T^2);
        q_seg(1) = q_start; q_seg(end) = q_end;
        qd_seg(1) = 0; qd_seg(end) = 0;
        qdd_seg(1) = 0; qdd_seg(end) = 0;
        return;
    end
    
    % 三段：加速(5次) + 匀速 + 减速(5次)
    delta = sign_dir * d_accel;  % 有符号加减速位移
    
    % ---- 1. 加速段：q_start, 0, 0 → q_start+delta, sign_dir*v_max, 0 ----
    % 局部 q_local(s)=delta*(2*s^3 - s^4)，s∈[0,1]，T=T_accel
    n1 = max(round(T_accel / Ts), 5);
    s1 = linspace(0, 1, n1)';
    t1 = s1 * T_accel;
    q1 = q_start + delta * (2*s1.^3 - s1.^4);
    qd1 = delta * (6*s1.^2 - 4*s1.^3) / T_accel;
    qdd1 = delta * (12*s1 - 12*s1.^2) / (T_accel^2);
    q1(1) = q_start; q1(end) = q_start + delta;
    qd1(1) = 0; qd1(end) = sign_dir * v_max_used;
    qdd1(1) = 0; qdd1(end) = 0;
    
    % ---- 2. 匀速段（至少 2 个点才插入，否则与加减速衔接点重复） ----
    T_const = (dist - 2*d_accel) / v_max_used;
    n2 = round(T_const / Ts) + 1;
    if n2 <= 1
        n2 = 0;
    end
    if n2 > 0
        t2 = T_accel + (0:(n2-1))' * Ts;
        q2 = (q_start + delta) + sign_dir * v_max_used * (t2 - T_accel);
        qd2 = sign_dir * v_max_used * ones(n2, 1);
        qdd2 = zeros(n2, 1);
        t2_end = T_accel + (n2-1)*Ts;
    else
        t2 = []; q2 = []; qd2 = []; qdd2 = [];
        t2_end = T_accel;
    end
    
    % ---- 3. 减速段：(q_end-delta, sign_dir*v_max, 0) → (q_end, 0, 0) ----
    % 局部 q_local(s)=delta*(2*s - 2*s^3 + s^4)，s∈[0,1]
    n3 = max(round(T_accel / Ts), 5);
    s3 = linspace(0, 1, n3)';
    t3 = t2_end + s3 * T_accel;
    q_local3 = delta * (2*s3 - 2*s3.^3 + s3.^4);
    q3 = (q_end - delta) + q_local3;
    qd3 = delta * (2 - 6*s3.^2 + 4*s3.^3) / T_accel;
    qdd3 = delta * (-12*s3 + 12*s3.^2) / (T_accel^2);
    q3(1) = q_end - delta; q3(end) = q_end;
    qd3(1) = sign_dir * v_max_used; qd3(end) = 0;
    qdd3(1) = 0; qdd3(end) = 0;
    
    % 拼接（相对时间从 0 开始）
    t_seg = [t1; t2; t3];
    q_seg = [q1; q2; q3];
    qd_seg = [qd1; qd2; qd3];
    qdd_seg = [qdd1; qdd2; qdd3];
end

%% 减速到零速度（位置不回到 0）：从 (q_start, qd_start) 平滑减到 (q_start+d, 0)，d 为减速过程位移
function [t_seg, q_seg, qd_seg, qdd_seg] = generate_decel_to_zero_velocity(q_start, qd_start, a_max_used, Ts)
    % 5 次多项式：局部 q_local(0)=0, qd_local(0)=qd_start → q_local(1)=d, qd_local(1)=0
    % d = sign(qd_start)*qd_start^2/(2*a_max), T = |qd_start|/a_max
    if abs(qd_start) < 1e-9
        t_seg = [0; Ts];
        q_seg = [q_start; q_start];
        qd_seg = [0; 0];
        qdd_seg = [0; 0];
        return;
    end
    d = sign(qd_start) * (qd_start^2 / (2 * a_max_used));
    T = abs(qd_start) / a_max_used;
    T = max(T, 0.2);
    n_points = max(round(T / Ts), 5);
    s = linspace(0, 1, n_points)';
    % q_local(s) = 2*d*s - 2*d*s^3 + d*s^4
    q_local = d * (2*s - 2*s.^3 + s.^4);
    dq_ds = d * (2 - 6*s.^2 + 4*s.^3);
    d2q_ds2 = d * (-12*s + 12*s.^2);
    t_seg = s * T;
    q_seg = q_start + q_local;
    qd_seg = dq_ds / T;
    qdd_seg = d2q_ds2 / (T^2);
    q_seg(1) = q_start; q_seg(end) = q_start + d;
    qd_seg(1) = qd_start; qd_seg(end) = 0;
    qdd_seg(1) = 0; qdd_seg(end) = 0;
end

%% 变速段：从 (q_start, qd_start) 匀加速/减速到 (q_start+d, qd_end)，同半段内档间连续变速（不经过 0）
function [t_seg, q_seg, qd_seg, qdd_seg] = generate_speed_change(q_start, qd_start, qd_end, a_max, Ts)
    % 匀加速：T = |qd_end-qd_start|/a_max, d = (qd_start+qd_end)*T/2
    if abs(qd_end - qd_start) < 1e-9
        t_seg = [0; Ts]; q_seg = [q_start; q_start];
        qd_seg = [qd_start; qd_start]; qdd_seg = [0; 0];
        return;
    end
    T = abs(qd_end - qd_start) / a_max;
    T = max(T, 0.15);
    d = (qd_start + qd_end) * T / 2;
    n_pts = max(round(T / Ts), 3);
    t_seg = linspace(0, T, n_pts)';
    qd_seg = qd_start + (qd_end - qd_start) * (t_seg / T);
    q_seg = q_start + qd_start * t_seg + 0.5 * (qd_end - qd_start) / T * t_seg.^2;
    qdd_seg = ((qd_end - qd_start) / T) * ones(n_pts, 1);
    q_seg(1) = q_start; q_seg(end) = q_start + d;
    qd_seg(1) = qd_start; qd_seg(end) = qd_end;
end

%% 线性斜坡（匀加速）：速度直线变化，无 S 形，用于 velocity_transition='linear'
function [t_seg, q_seg, qd_seg, qdd_seg] = generate_linear_ramp(q_start, q_end, qd_start, qd_end, Ts)
    % 从 (q_start, qd_start) 匀加速到 (q_end, qd_end)，加速度恒定，速度呈直线
    dq = q_end - q_start;
    qd_sum = qd_start + qd_end;
    if abs(qd_sum) < 1e-9
        T = sqrt(2 * abs(dq) / 1e-6);  % 避免除零，用极小加速度估算
    else
        T = abs(2 * dq / qd_sum);  % 匀加速：dq = 0.5*(qd_start+qd_end)*T
    end
    T = max(T, Ts * 2);  % 至少 2 个采样点
    a = (qd_end - qd_start) / T;
    n_points = max(round(T / Ts), 2);
    t_seg = linspace(0, T, n_points)';
    qd_seg = qd_start + a * t_seg;
    q_seg = q_start + qd_start * t_seg + 0.5 * a * t_seg.^2;
    qdd_seg = a * ones(n_points, 1);
    q_seg(1) = q_start;
    q_seg(end) = q_end;
    qd_seg(1) = qd_start;
    qd_seg(end) = qd_end;
end
