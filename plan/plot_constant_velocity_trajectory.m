% plot_constant_velocity_trajectory  绘制匀速轨迹的位置、速度、加速度图
%
% 功能：读取生成的匀速轨迹CSV文件，绘制位置、速度、加速度时序图
%
% 使用方法:
%   plot_constant_velocity_trajectory()
%   plot_constant_velocity_trajectory('csv_file', 'constant_velocity_0.3rads_4s.csv')
%   plot_constant_velocity_trajectory('csv_file', 'constant_velocity_0.3rads_4s.csv', 'joint_name', 'l_leg_hip_yaw_joint')
%
% 输入参数（可选）:
%   'csv_file'   - CSV轨迹文件路径，默认查找当前目录下的 constant_velocity_*.csv
%   'joint_name' - 要绘制的关节名，默认自动检测（第一个非零列）

function plot_constant_velocity_trajectory(varargin)

p = inputParser;
addParameter(p, 'csv_file', '', @ischar);
addParameter(p, 'joint_name', '', @ischar);
parse(p, varargin{:});
opts = p.Results;

%% 1. 定位CSV文件（默认先查 data/ 与 data/plan，再查当前目录）
if isempty(opts.csv_file)
    for search_dir = {get_data_dir(), get_data_dir('plan'), pwd}
        csv_files = dir(fullfile(search_dir{1}, 'constant_velocity_*.csv'));
        if ~isempty(csv_files)
            opts.csv_file = fullfile(search_dir{1}, csv_files(1).name);
            fprintf('自动找到文件: %s\n', opts.csv_file);
            break;
        end
    end
    if isempty(opts.csv_file)
        error('未找到 constant_velocity_*.csv 文件，请将数据放入 data/ 或指定 csv_file 参数');
    end
end

if ~exist(opts.csv_file, 'file')
    error('文件不存在: %s', opts.csv_file);
end

%% 2. 读取CSV文件
fprintf('读取轨迹文件: %s\n', opts.csv_file);
data = readmatrix(opts.csv_file);

% 读取表头
fid = fopen(opts.csv_file, 'r');
header_line = fgetl(fid);
fclose(fid);
header = strsplit(header_line, ',');

% 提取时间列
time = data(:, 1);

% 确定采样周期
if length(time) > 1
    Ts = time(2) - time(1);
else
    Ts = 0.005;  % 默认200Hz
    warning('无法确定采样周期，使用默认值 %.4f s', Ts);
end

%% 3. 确定要绘制的关节
if isempty(opts.joint_name)
    % 自动检测第一个非零列（除了time列）
    for j = 2:size(data, 2)
        if any(abs(data(:, j)) > 1e-6)
            joint_idx = j;
            if j <= length(header)
                opts.joint_name = header{j};
            else
                opts.joint_name = sprintf('joint_%d', j-1);
            end
            break;
        end
    end
    if isempty(opts.joint_name)
        error('未找到非零关节列');
    end
else
    % 根据关节名查找列索引
    joint_idx = find(strcmp(header, opts.joint_name), 1);
    if isempty(joint_idx)
        error('未找到关节: %s', opts.joint_name);
    end
end

fprintf('绘制关节: %s (列索引: %d)\n', opts.joint_name, joint_idx);

% 提取关节位置
q = data(:, joint_idx);

%% 4. 计算速度和加速度（数值微分）
% 速度：qd = diff(q) / Ts
qd = [0; diff(q) / Ts];

% 加速度：qdd = diff(qd) / Ts
qdd = [0; diff(qd) / Ts];

% 可选：平滑加速度（减少噪声）
if length(qdd) > 5
    % 简单的移动平均平滑
    window_size = min(5, floor(length(qdd)/10));
    if window_size > 1
        qdd_smooth = movmean(qdd, window_size);
    else
        qdd_smooth = qdd;
    end
else
    qdd_smooth = qdd;
end

%% 5. 绘制图形
figure('Name', sprintf('匀速轨迹: %s', opts.joint_name), ...
       'Position', [100, 100, 1200, 800]);

% 子图1：位置
subplot(3, 1, 1);
plot(time, q * 180/pi, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel('位置 (deg)');
title(sprintf('%s - 位置轨迹', opts.joint_name), 'FontSize', 12, 'FontWeight', 'bold');
legend(sprintf('位置 (范围: [%.2f, %.2f] deg)', min(q)*180/pi, max(q)*180/pi), ...
       'Location', 'best');

% 子图2：速度
subplot(3, 1, 2);
plot(time, qd * 180/pi, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel('速度 (deg/s)');
title(sprintf('%s - 速度轨迹', opts.joint_name), 'FontSize', 12, 'FontWeight', 'bold');
legend(sprintf('速度 (范围: [%.2f, %.2f] deg/s)', min(qd)*180/pi, max(qd)*180/pi), ...
       'Location', 'best');
% 添加理论速度线（如果速度恒定）
if abs(std(qd)) < 0.1  % 如果速度变化很小，认为是匀速
    hold on;
    yline(mean(qd)*180/pi, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('平均速度: %.2f deg/s', mean(qd)*180/pi));
end

% 子图3：加速度
subplot(3, 1, 3);
plot(time, qdd_smooth, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel('加速度 (rad/s²)');
title(sprintf('%s - 加速度轨迹（数值微分，已平滑）', opts.joint_name), ...
      'FontSize', 12, 'FontWeight', 'bold');
legend(sprintf('加速度 (范围: [%.3f, %.3f] rad/s²)', min(qdd_smooth), max(qdd_smooth)), ...
       'Location', 'best');
% 添加零线
hold on;
yline(0, 'k--', 'LineWidth', 0.5, 'DisplayName', '零线');

sgtitle(sprintf('匀速轨迹分析: %s (采样: %.0f Hz)', opts.joint_name, 1/Ts), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% 6. 打印统计信息
fprintf('\n========== 轨迹统计信息 ==========\n');
fprintf('文件: %s\n', opts.csv_file);
fprintf('关节: %s\n', opts.joint_name);
fprintf('采样频率: %.0f Hz (Ts = %.4f s)\n', 1/Ts, Ts);
fprintf('总时长: %.2f s\n', time(end));
fprintf('数据点数: %d\n', length(time));
fprintf('\n位置统计:\n');
fprintf('  范围: [%.4f, %.4f] rad ([%.2f, %.2f] deg)\n', ...
    min(q), max(q), min(q)*180/pi, max(q)*180/pi);
fprintf('  变化量: %.4f rad (%.2f deg)\n', max(q)-min(q), (max(q)-min(q))*180/pi);
fprintf('\n速度统计:\n');
fprintf('  范围: [%.4f, %.4f] rad/s ([%.2f, %.2f] deg/s)\n', ...
    min(qd), max(qd), min(qd)*180/pi, max(qd)*180/pi);
fprintf('  平均速度: %.4f rad/s (%.2f deg/s)\n', mean(qd), mean(qd)*180/pi);
fprintf('  速度标准差: %.4f rad/s\n', std(qd));
fprintf('\n加速度统计:\n');
fprintf('  范围: [%.4f, %.4f] rad/s²\n', min(qdd_smooth), max(qdd_smooth));
fprintf('  平均加速度: %.4f rad/s²\n', mean(qdd_smooth));
fprintf('  加速度标准差: %.4f rad/s²\n', std(qdd_smooth));
fprintf('====================================\n');

end
