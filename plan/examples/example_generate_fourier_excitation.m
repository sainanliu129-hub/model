% example_generate_fourier_excitation  傅里叶激励轨迹生成示例
%
% 与多速度档位往返轨迹互补，适用于 Part B 完整动力学验证。
%
% 用法：先执行项目根目录 addpaths，再运行本脚本。

clear; clc;

Ts = 0.002;              % 500 Hz
duration = 20;           % 傅里叶段时长（秒）
n_harmonics = 5;         % 谐波数 N
n_periods = 1;           % 整周期数（保证 q(0)=q(T)=0）
transition_time = 1.0;   % 起止过渡段时长（秒）

generate_fourier_excitation_trajectory(...
    'Ts', Ts, ...
    'duration', duration, ...
    'n_harmonics', n_harmonics, ...
    'n_periods', n_periods, ...
    'transition_time', transition_time, ...
    'output_file', fullfile(get_build_dir('plan'), 'fourier_excitation_trajectory.csv'));

fprintf('傅里叶激励轨迹已生成（build/plan/）\n');
