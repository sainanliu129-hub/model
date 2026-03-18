% example_view_trajectory  轨迹 CSV 读取与可视化示例
%
% 演示：read_trajectory_csv 读取、visualize_trajectory 绘图。
% 用法：先执行项目根目录 addpaths，或将 csv_file 改为实际轨迹文件后运行。

clear; clc;

if isempty(which('read_trajectory_csv'))
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'utility_function'));
end

csv_file = fullfile(get_data_dir(), 'walkrun_target_trajectory.csv');
if ~exist(csv_file, 'file')
    d = dir(fullfile(get_build_dir('plan'), '*.csv'));
    if ~isempty(d), csv_file = fullfile(d(1).folder, d(1).name); end
end

if ~exist(csv_file, 'file')
    fprintf('未找到轨迹文件，请将 csv_file 改为实际路径\n');
    return;
end

% 使用 read_trajectory_csv 读取（内部等价于 try readmatrix NumHeaderLines 1 / catch readtable）
[time, q_all] = read_trajectory_csv(csv_file);
fprintf('数据点数: %d, 关节数: %d, 时间范围: [%.3f, %.3f] s\n', ...
    length(time), size(q_all, 2), time(1), time(end));

% 可视化
fprintf('使用 visualize_trajectory 绘图...\n');
visualize_trajectory(csv_file, 'joints', 1:min(12, size(q_all,2)), 'save_fig', false);
