% example_generate_constant_velocity_03rads  生成0.3 rad/s往返匀速轨迹示例
%
% 功能：生成速度为0.3 rad/s的往返匀速轨迹，用于验证机器人运动
%       规划方式与 plan/examples/example_generate_urdf_validation_trajectory 方式1相同：
%       - 自动从URDF读取关节限位
%       - 运动范围 = URDF限位的90%
%       - 生成往返轨迹（正转、回程、反转）
%
% 使用方法:
%   example_generate_constant_velocity_03rads()
%
% 输出:
%   constant_velocity_0.3rads_*.csv - 0.3 rad/s往返匀速轨迹文件

clear; clc;

%% 添加路径
base = fileparts(mfilename('fullpath'));
addpath(genpath(base));

%% 生成0.3 rad/s匀速轨迹
fprintf('===== 生成0.3 rad/s匀速轨迹 =====\n');

% 参数设置
velocity = 0.3;        % 速度：0.3 rad/s（低速档）
duration = [];         % 持续时间：空值表示根据URDF限位自动计算
joint_name = 'l_leg_hip_yaw_joint';  % 关节名（默认左腿髋关节yaw）

% 生成轨迹（运动范围自动设为URDF限位的0.9倍）
generate_constant_velocity_trajectory(...
    'velocity', velocity, ...
    'duration', duration, ...  % 自动根据URDF限位和速度计算
    'joint_name', joint_name, ...
    'output_dir', base);  % 保存到脚本所在目录

fprintf('\n轨迹生成完成！\n');
fprintf('参数: 速度=%.1f rad/s, 往返轨迹（正转+回程+反转）\n', velocity);
fprintf('运动范围: URDF限位的90%%, 采样=200 Hz\n');
fprintf('用途: 验证机器人运动（固定低速0.3 rad/s）\n');
