% example_compare_single_joint_torque  单关节往返轨迹对齐查看模型输出力矩和实际力矩误差示例
%
% 本脚本演示如何使用 compare_single_joint_torque_aligned 进行单关节往返轨迹对齐，
% 并对比模型输出力矩和实际力矩误差。
%
% 注意：
%   - 函数会自动将文件分成12段，每段对应一个关节的往返轨迹
%   - 对每个关节段分别进行往返轨迹对齐
%   - 如果任何关节段出现重叠错误，程序会直接退出
%
% 使用方法：
%   1. 准备数据文件：包含12个关节往返轨迹的CSV文件（格式见 read_leg_joint_csv.m）
%   2. 运行本脚本或直接调用 compare_single_joint_torque_aligned
%   3. 查看生成的对比图和误差统计

clear;
clc;
close all;

%% 示例1：使用默认设置（自动查找CSV文件）
fprintf('========== 示例1：使用默认设置 ==========\n');
% compare_single_joint_torque_aligned();

%% 示例2：指定数据文件（动力学参数辨识数据放在 data/dynamics/ 或 data/dynamics/single_pos/）
fprintf('\n========== 示例2：指定数据文件 ==========\n');
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
dd = get_data_dir('dynamics');   % data/dynamics
csv_file = fullfile(dd, 'single_pos', 'PD-M1-v0_multi_joint_20260206-205247.csv');
if ~exist(csv_file, 'file')
    csv_file = fullfile(dd, 'PD-M1-v0_multi_joint_20260206-205247.csv');  % 或直接放在 data/dynamics 下
end
if exist(csv_file, 'file')
    compare_single_joint_torque_aligned('csv_file', csv_file);
else
    fprintf('数据文件不存在，请将 CSV 放入 %s 或 %s\n', fullfile(dd, 'single_pos'), dd);
end

%% 示例3：自定义对齐范围和参数
fprintf('\n========== 示例3：自定义对齐范围和参数 ==========\n');
% compare_single_joint_torque_aligned(...
%     'csv_file', csv_file, ...
%     'forward_range', [0.1, 0.4], ...    % 正向段范围 [起始比例, 结束比例]（默认 [0.1, 0.4]）
%     'qdd_smooth', 5, ...                % 加速度平滑点数
%     'plot_all_joints', true);           % 是否绘制所有关节

%% 示例4：只绘制第一个关节
fprintf('\n========== 示例4：只绘制第一个关节 ==========\n');
% compare_single_joint_torque_aligned(...
%     'csv_file', csv_file, ...
%     'plot_all_joints', false);

fprintf('\n========== 完成 ==========\n');
fprintf('使用说明:\n');
fprintf('  1. 取消注释上面的示例代码，根据实际情况修改参数\n');
fprintf('  2. 确保CSV文件格式正确（见 read_leg_joint_csv.m）\n');
fprintf('  3. 文件会被自动分成12段，每段对应一个关节的往返轨迹\n');
fprintf('  4. 如果任何关节段出现重叠错误，程序会直接退出\n');
fprintf('  5. 运行脚本查看对比结果\n');
fprintf('  6. 生成的图表包括：\n');
fprintf('     - 对齐后的实际力矩 vs 模型全动力学力矩\n');
fprintf('     - 对齐后的实际力矩 vs 模型重力项\n');
fprintf('     - 误差分布直方图\n');
