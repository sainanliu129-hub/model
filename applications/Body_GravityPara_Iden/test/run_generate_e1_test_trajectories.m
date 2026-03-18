% run_generate_e1_test_trajectories  调用 generate_e1_test_trajectories 生成 E1 测试轨迹
%
% 用法：在 MATLAB 中直接运行本脚本（F5 或命令行输入 run_generate_e1_test_trajectories）
% 生成轨迹会保存在当前脚本所在目录，并导出 CSV、绘制 q/qd/qdd 图。

clear; clc; close all;

% 保证能找到 generate_e1_test_trajectories 及同目录下的子函数
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

% 输出目录：本模块 build/（不存在则创建）
csv_dir = fullfile(app_root, 'build');
if ~exist(csv_dir, 'dir')
    mkdir(csv_dir);
end

robot_name = 'E1_update';
% 调用生成轨迹：导出 CSV、保存 MAT、绘图
traj = generate_e1_test_trajectories( ...
    'robot_name',   robot_name, ...
    'export_csv',   true, ...
    'csv_dir',      csv_dir, ...
    'save_mat',     fullfile(csv_dir, 'e1_test_traj.mat'), ...
    'plot_traj',    true);

fprintf('\n===== 生成完成 =====\n');
fprintf('  轨迹结构体: traj（已在工作区）\n');
fprintf('  CSV 文件:   %s\n', fullfile(csv_dir, ['traj_', robot_name, '_all_joints.csv']));
fprintf('  MAT 文件:   %s\n', fullfile(csv_dir, 'e1_test_traj.mat'));
