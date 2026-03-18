% 示例：同一数据文件做两种对比
%   1) 逆动力学对比：数据力矩 vs ID(q,qd,qdd) 模型力矩
%   2) 正动力学对比：用 FD(q,qd,tau) 积分得到的 q/qd 与数据 q/qd 对比（验证 FD+积分与 Gazebo 是否一致）
%
% 数据文件为必选输入，无默认路径。

clc;
clear;
close all;

% 数据文件：路径相对于本脚本所在目录（与当前工作目录 pwd 无关）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
csv_file = fullfile(app_root, 'data','跑步', 'PD-M1-v0_multi_joint_20260226-182548.csv');   % 可改为你的相对路径或文件名
% csv_file_reverse = fullfile(app_root, 'data','跑步', 'PD-M1-v0_multi_joint_20260226-182352.csv');   % 可改为你的相对路径或文件名
%% 一、逆动力学对比：三条线 — 实际关节力矩、动力学、动力学+摩擦力矩（仅膝 L4/R4 加摩擦）
% compare_torque_sim_vs_id(csv_file);
% compare_torque_sim_vs_id(csv_file, 'qd_smooth_then_qdd', true);
% compare_torque_sim_vs_id('csv_file', csv_file, 'qd_qdd_from_q', true);
% compare_torque_sim_vs_id('csv_file', csv_file, 'qd_smooth_then_qdd', true, 'fit_tau_err_qdd', true, 'save_png', true);
% compare_torque_sim_vs_id(csv_file, 'align', true, 'csv_file_reverse', csv_file_reverse)
compare_torque_sim_vs_id('csv_file', csv_file, 'compare_qdd_diff_vs_fd', true, 'save_png', true);
% 可选：compare_torque_sim_vs_id(csv_file, 'align', true);
% [rmse_no, rmse_align, data] = compare_torque_sim_vs_id(csv_file);

%% 二、正动力学对比：FD 积分得到的 q/qd vs 数据 q/qd（同一文件）
% compare_forward_dynamics_vs_measured(csv_file, 'chain', 'both_legs', 'save_png', true);
% 用仿真上一拍（纯正动力学，可能飞）
% compare_forward_dynamics_vs_measured(csv_file, 'chain', 'both_legs', 'use_sim_state', true);
% 可选：compare_forward_dynamics_vs_measured(csv_file, 'chain', 'left_leg');
% 同时对比「力矩减膝摩擦后再 FD」：仅膝关节有摩擦，参数 τ_s=3.12, τ_c=2.0267, b=0.4441
% compare_forward_dynamics_vs_measured(csv_file, 'chain', 'both_legs', 'compare_with_friction_comp', true);
