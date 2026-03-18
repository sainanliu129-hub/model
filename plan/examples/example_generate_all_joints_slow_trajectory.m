% example_generate_all_joints_slow_trajectory  所有关节依次 Part A 轨迹生成示例
%
% 为每个腿关节依次生成 Part A（正反转匀速）轨迹，速度比例为最大速度的 0.01。
%
% 用法：先执行项目根目录 addpaths，再运行本脚本。

clear;
clc;
close all;

%% 参数设置
Ts = 0.005;  % 采样周期
velocity_ratio = 0.01;  % 最大速度的 0.01 倍

fprintf('========================================\n');
fprintf('所有关节依次运动轨迹生成（Part A）\n');
fprintf('========================================\n\n');

base = fileparts(mfilename('fullpath'));
addpath(fullfile(base, '..', '..', 'utility_function'));

urdf_file = get_e1_urdf_path();
[all_joint_names, ~] = get_e1_leg_joint_names();
[joint_velocity_max, ~] = read_e1_leg_limits_from_urdf(urdf_file);

fprintf('URDF: %s\n', urdf_file);
for i = 1:length(all_joint_names)
    fprintf('  %s: v_max=%.2f rad/s\n', all_joint_names{i}, joint_velocity_max(all_joint_names{i}));
end

%% 为每个关节生成 Part A 轨迹
fprintf('\n========== 生成 Part A 轨迹 ==========\n');
for i = 1:length(all_joint_names)
    joint_name = all_joint_names{i};
    v_max = joint_velocity_max(joint_name);
    velocity = v_max * velocity_ratio;
    fprintf('关节 %d/%d: %s, 使用速度 %.4f rad/s\n', i, length(all_joint_names), joint_name, velocity);
    generate_urdf_validation_trajectory('part_a', joint_name, ...
        'Ts', Ts, ...
        'velocity', velocity, ...
        'urdf_file', urdf_file);
end

fprintf('\n========== 完成 ==========\n');
fprintf('所有关节的 Part A 轨迹已生成（默认在 build/plan/）\n');
