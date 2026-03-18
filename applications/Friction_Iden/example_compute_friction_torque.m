% example_compute_friction_torque  已知一组速度与摩擦参数，计算摩擦力矩
%
% 摩擦模型（与 identify_joint_friction 一致）：
%   qd > 0:  τ_f = τ_c_pos + b × qd
%   qd < 0:  τ_f = τ_c_neg + b × qd
%   qd = 0:  τ_f = 0
% 对称时可写为 τ_f = τ_c×sign(qd) + b×qd。
%
% 用法：先运行 run_full_iden_three_files 得到 Result_friction，或直接给定参数后运行本脚本。

clear; clc; close all;

%% 1. 摩擦参数（可从辨识结果填入，或手动给定）
% 方式 A：若已运行过 run_full_iden_three_files，可在此加载或直接赋值
% Result_friction = identify_friction_from_trajectory(...);
% tau_c_pos = Result_friction.tau_c_pos;
% tau_c_neg = Result_friction.tau_c_neg;
% b         = Result_friction.b;

% 方式 B：手动给定（示例值）
tau_c_pos = 1.8;   % N·m，正向库伦
tau_c_neg = -1.7;  % N·m，负向库伦（通常为负）
b         = 0.05;  % N·m·s/rad，粘滞系数

%% 2. 已知的一组速度（rad/s）
qd_list = [-12; -8; -4; -1; -0.1; 0; 0.1; 1; 4; 8; 12];

%% 3. 计算摩擦力矩（调用 robot_algorithm/Friction 中的模型）
cur = fileparts(mfilename('fullpath'));
rp = fullfile(cur, '..', '..', 'robot_algorithm');
if exist(rp, 'dir') && isempty(which('compute_friction_torque'))
    addpath(genpath(rp));
end

% 不对称模型：用 TauCPos、TauCNeg
tau_f = compute_friction_torque(qd_list, 0, b, 'TauCPos', tau_c_pos, 'TauCNeg', tau_c_neg);

% 若只有对称 τ_c，可改为：tau_f = compute_friction_torque(qd_list, tau_c, b);

%% 4. 打印与绘图
fprintf('速度 qd (rad/s)  →  摩擦力矩 τ_f (N·m)\n');
fprintf('----------------------------------------\n');
for k = 1:length(qd_list)
    fprintf('  %+7.2f          %+7.4f\n', qd_list(k), tau_f(k));
end

figure('Name', '摩擦力矩模型');
plot(qd_list, tau_f, 'bo-', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on;
xlabel('角速度 qd (rad/s)');
ylabel('摩擦力矩 \tau_f (N\cdotm)');
title(sprintf('摩擦模型: \\tau_{c+}=%.2f, \\tau_{c-}=%.2f, b=%.4f', tau_c_pos, tau_c_neg, b));
