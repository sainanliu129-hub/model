% example_generate_multi_speed_trajectory  多速度档位轨迹生成示例
%
% 两种模式：
%   multi_speed: 3个速度档位（0.3、0.6、0.9倍最大速度）
%   low_speed:   固定速度 0.3 rad/s（单档）
%
% 用法：先执行项目根目录 addpaths，再运行本脚本。

clear;
clc;
close all;

%% 参数设置
Ts = 0.005;  % 采样周期
mode = 'low_speed';  % 'multi_speed' 或 'low_speed'

fprintf('========================================\n');
fprintf('多速度档位往返匀速轨迹生成\n');
fprintf('========================================\n\n');
fprintf('采样周期: %.4f s (%.0f Hz)\n', Ts, 1/Ts);
fprintf('模式: %s\n\n', mode);

%% 确保 utility_function 在路径（get_e1_urdf_path 等）
base = fileparts(mfilename('fullpath'));
addpath(fullfile(base, '..', '..', 'utility_function'));

%% 生成轨迹
if strcmp(mode, 'multi_speed')
    fprintf('--- 模式1：多档测试 ---\n');
    generate_multi_speed_trajectory(...
        'Ts', Ts, ...
        'urdf_file', '', ...
        'speed_ratios', [0.3, 0.6, 0.9], ...
        'output_file', fullfile(get_build_dir('plan'), 'dyn_para_test_traj.csv'));
elseif strcmp(mode, 'low_speed')
    fprintf('--- 模式2：低速 0.3 rad/s ---\n');
    urdf_file = get_e1_urdf_path();
    [all_joint_names, ~] = get_e1_leg_joint_names();
    [joint_velocity_max, ~] = read_e1_leg_limits_from_urdf(urdf_file);
    v_max_min = min(cellfun(@(name) joint_velocity_max(name), all_joint_names));
    speed_ratio = 0.3 / v_max_min;
    fprintf('最小最大速度: %.2f rad/s, 速度比例: %.4f\n', v_max_min, speed_ratio);
    generate_multi_speed_trajectory(...
        'Ts', Ts, ...
        'urdf_file', urdf_file, ...
        'speed_ratios', [speed_ratio], ...
        'output_file', fullfile(get_build_dir('plan'), 'dyn_para_test_traj.csv'));
else
    error('未知模式: %s，请使用 ''multi_speed'' 或 ''low_speed''', mode);
end

fprintf('\n========== 完成 ==========\n');
fprintf('轨迹文件已生成，可用于真机测试\n');
