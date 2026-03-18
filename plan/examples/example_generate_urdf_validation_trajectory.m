% example_generate_urdf_validation_trajectory  URDF验证轨迹生成示例
%
% 演示生成用于URDF动力学参数验证的测试轨迹：
%   Part A: 正反转匀速轨迹（重力验证）
%   Part B: 带加速度轨迹（完整动力学力矩验证）
%
% 用法：先执行项目根目录 addpaths，再运行本脚本。

clear;
clc;
close all;

%% 参数设置
joint_name = 'l_leg_hip_yaw_joint';  % 要测试的关节名称
Ts = 0.005;  % 采样周期 (500 Hz)

fprintf('========================================\n');
fprintf('URDF动力学参数验证轨迹生成\n');
fprintf('========================================\n\n');
fprintf('目标关节: %s\n', joint_name);
fprintf('采样周期: %.4f s (%.0f Hz)\n\n', Ts, 1/Ts);

%% Part A: 正反转匀速轨迹
fprintf('========== Part A: 正反转匀速轨迹 ==========\n');
velocity = 0.2;      % 匀速速度 rad/s（建议0.1~0.3）

generate_urdf_validation_trajectory('part_a', joint_name, ...
    'Ts', Ts, ...
    'velocity', velocity);

fprintf('\n');

%% Part B: 带加速度轨迹（正弦 + 扫频示例）
fprintf('========== Part B: 带加速度轨迹 ==========\n');

% 正弦轨迹
generate_urdf_validation_trajectory('part_b', joint_name, ...
    'Ts', Ts, ...
    'trajectory_shape', 'sine', ...
    'amplitude', 0.5, ...
    'frequency', 0.1, ...
    'duration', 30);

% 扫频轨迹（可选）
% generate_urdf_validation_trajectory('part_b', joint_name, ...
%     'Ts', Ts, ...
%     'trajectory_shape', 'chirp', ...
%     'amplitude', 0.5, ...
%     'freq_start', 0.05, ...
%     'freq_end', 0.5, ...
%     'duration', 30);

fprintf('\n========== 完成 ==========\n');
fprintf('轨迹文件已生成（默认在 build/plan/），可用于真机测试\n');
