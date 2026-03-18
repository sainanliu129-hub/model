% example_plot_constant_velocity_trajectory  绘制匀速轨迹示例
%
% 功能：读取并绘制生成的匀速轨迹文件的位置、速度、加速度图
%
% 使用方法:
%   example_plot_constant_velocity_trajectory()

clear; clc; close all;

%% 添加路径
base = fileparts(mfilename('fullpath'));
addpath(genpath(base));

%% 方法1：自动查找并绘制（推荐）
fprintf('===== 方法1：自动查找轨迹文件并绘制 =====\n');
plot_constant_velocity_trajectory();

%% 方法2：指定文件路径
% fprintf('\n===== 方法2：指定文件路径 =====\n');
% csv_file = fullfile(base, 'constant_velocity_0.3rads_4s.csv');
% plot_constant_velocity_trajectory('csv_file', csv_file);

%% 方法3：指定文件和关节名
% fprintf('\n===== 方法3：指定文件和关节名 =====\n');
% csv_file = fullfile(base, 'constant_velocity_0.3rads_4s.csv');
% plot_constant_velocity_trajectory('csv_file', csv_file, 'joint_name', 'l_leg_hip_yaw_joint');

fprintf('\n绘图完成！\n');
