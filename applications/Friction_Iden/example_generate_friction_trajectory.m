% example_generate_friction_trajectory  摩擦力辨识轨迹生成示例脚本
%
% 本脚本演示如何使用 generate_friction_trajectory 生成摩擦力辨识轨迹
% 输出：每个关节 2 个文件（速度/加速度分开放），格式均为 time + 12 关节列；500 Hz；无突变
%
% 使用方法：
%   1. 运行脚本生成轨迹文件（默认写到当前目录）
%   2. 将对应关节的轨迹文件发送给控制器执行
%   3. 采集数据后进行处理

clear;
clc;
close all;

%% 参数设置
% 最大速度从 URDF 自动读取
a_max = 300.0;      % 关节最大加速度 rad/s²

% 左腿所有6个关节
left_leg_joints = {
    'l_leg_hip_yaw_joint';
    'l_leg_hip_roll_joint';
    'l_leg_hip_pitch_joint';
    'l_leg_knee_joint';
    'l_leg_ankle_pitch_joint';
    'l_leg_ankle_roll_joint'
};

%% 示例1：生成左腿所有关节的轨迹（每关节两文件，500 Hz）
fprintf('========== 示例1：生成左腿所有关节的摩擦力辨识轨迹（每关节速度/加速度分文件）==========\n');
generate_friction_trajectory('joint_names', left_leg_joints, 'a_max', a_max);

%% 示例2：指定输出目录（可选）
fprintf('\n========== 示例2：指定输出目录（可选）==========\n');
% generate_friction_trajectory('joint_names', left_leg_joints, ...
%     'output_dir', 'trajectory_output', 'a_max', a_max);

%% 加载并查看轨迹数据（示例）
fprintf('\n========== 加载轨迹数据示例 ==========\n');

example_joint = 'l_leg_hip_yaw_joint';
vel_file = [example_joint '_velocity_trajectory_500Hz.csv'];
acc_file = [example_joint '_acceleration_trajectory_500Hz.csv'];

for f = {vel_file, acc_file}
    fn = f{1};
    if exist(fn, 'file')
        fprintf('---------- %s ----------\n', fn);
        try
            data_csv = readmatrix(fn);
            if size(data_csv, 1) < 2
                fprintf('  数据点不足\n');
            else
                fprintf('  %d 行 × %d 列 (time + 12 关节), 时长 %.2f s\n', ...
                    size(data_csv, 1), size(data_csv, 2), data_csv(end, 1));
            end
        catch ME
            fprintf('  读取出错: %s\n', ME.message);
        end
    else
        fprintf('未找到: %s（请先运行示例1）\n', fn);
    end
end

fprintf('\n========== 完成 ==========\n');
fprintf('输出说明:\n');
fprintf('  1. 速度与加速度分开放：每个关节有 *_velocity_* 与 *_acceleration_* 两个文件\n');
fprintf('  2. 每个文件格式: time,l_leg_hip_yaw_joint,...,r_leg_ankle_roll_joint（仅当前关节列非零）\n');
fprintf('  3. 采样频率: 500 Hz；关节角、速度、加速度无突变\n');
fprintf('  4. 最大速度: 从 URDF 自动读取\n');
