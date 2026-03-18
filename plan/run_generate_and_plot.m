% run_generate_and_plot  生成多速度档位轨迹并绘制速度
%
% 完整流程：生成轨迹 -> 绘制速度验证
%
% 用法：直接运行此脚本

clear; clc; close all;

%% 1. 生成多速度档位轨迹
fprintf('========================================\n');
fprintf('步骤1：生成多速度档位轨迹\n');
fprintf('========================================\n\n');

generate_multi_speed_trajectory(...
    'Ts', 0.002, ...
    'speed_ratios', [0.3, 0.6, 0.9], ...
    'accel_ratio', 1.0, ...  % 增加到1.0以获得更大的加速度（有助于在有限距离内达到高速度）
    'output_file', 'my_trajectory.csv' ...
);

fprintf('\n');

%% 2. 绘制速度（验证是否达到期望速度）
fprintf('========================================\n');
fprintf('步骤2：绘制速度验证\n');
fprintf('========================================\n\n');

csv_file = 'my_trajectory.csv';

% 根据URDF中的最大速度，计算各关节的期望速度档位
% l_leg_ankle_pitch_joint (v_max=12.46): [3.738, 7.476, 11.214]
% l_leg_ankle_roll_joint (v_max=12.46): [3.738, 7.476, 11.214]
% l_leg_knee_joint (v_max=13.18): [3.954, 7.908, 11.862]
% l_leg_hip_yaw_joint (v_max=16.75): [5.025, 10.050, 15.075]
% l_leg_hip_roll_joint (v_max=20.10): [6.030, 12.060, 18.090]
% l_leg_hip_pitch_joint (v_max=20.10): [6.030, 12.060, 18.090]
% 右腿关节类似

% 使用 l_leg_ankle_pitch_joint 的期望速度作为示例
expected_speeds_ankle = [3.738, 7.476, 11.214];  % 30%, 60%, 90% of 12.46

fprintf('绘制所有关节速度（使用 l_leg_ankle_pitch_joint 的期望速度作为参考）...\n');
plot_multi_speed_velocity(csv_file, ...
    'expected_speeds', expected_speeds_ankle, ...
    'joints', 1:12);

fprintf('\n提示：如果某个关节的最大速度与期望值不同，\n');
fprintf('      可以单独绘制该关节并指定其对应的期望速度。\n');
fprintf('      例如：plot_multi_speed_velocity(csv_file, ''expected_speeds'', [5.025, 10.050, 15.075], ''joints'', 1);\n');

fprintf('\n========================================\n');
fprintf('完成！\n');
fprintf('========================================\n');
