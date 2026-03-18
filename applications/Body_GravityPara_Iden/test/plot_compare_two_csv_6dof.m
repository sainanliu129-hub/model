% plot_compare_two_csv_6dof  绘制两个 CSV 的关节角、角速度、力矩对比及误差图（左腿 6 关节）
%
% 用法：直接运行本脚本，或在命令行中：
%   run('applications/Body_GravityPara_Iden/plot_compare_two_csv_6dof.m')
% 如需更换数据文件，修改下面的 file_actual 和 file_sim 路径即可。
%
% 对比内容：
%   1) 关节角 q：期望、实际、仿真 及 误差(实际-仿真)
%   2) 角速度 qd：实际、仿真 及 误差
%   3) 力矩 torque：实际、仿真 及 误差
% 并在命令窗口输出各关节误差统计：最大值、RMSE、MAE、均值、标准差。

clc;
close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
ensure_body_gravity_para_iden_path();

% 实际数据、仿真数据（可改为你的文件名）
file_actual = fullfile(app_root, 'data', '跑步', 'PD-M1-v0_multi_joint_20260226-181712.csv');
file_sim    = fullfile(app_root, 'data', '跑步', 'PD-M1-v0_multi_joint_20260227-071606_sim.csv');

% PNG 输出目录
out_dir = fullfile(app_root, 'fig_png');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

data1 = read_leg_joint_csv(file_actual);
data2 = read_leg_joint_csv(file_sim);

% 取相同长度以便做差
n = min(size(data1.pos_leg_l, 1), size(data2.pos_leg_l, 1));
t = data1.time(1:n);

% 左腿数据（可改为 _r 做右腿）
pos_l_1 = data1.pos_leg_l(1:n, :);
pos_l_2 = data2.pos_leg_l(1:n, :);
vel_l_1 = data1.vel_leg_l(1:n, :);
vel_l_2 = data2.vel_leg_l(1:n, :);
tau_l_1 = data1.torque_leg_l(1:n, :);
tau_l_2 = data2.torque_leg_l(1:n, :);
cmd_l_1 = data1.cmd_leg_l(1:n, :);
cmd_l_2 = data2.cmd_leg_l(1:n, :);

% 误差（实际 - 仿真）
err_q   = pos_l_1 - pos_l_2;
err_qd  = vel_l_1 - vel_l_2;
err_tau = tau_l_1 - tau_l_2;

%% 误差统计（最大值、均方根、平均绝对误差等）
joint_labels = {'L1', 'L2', 'L3', 'L4', 'L5', 'L6'};
stats_q   = compute_err_stats(err_q,   joint_labels, '关节角 q / rad');
stats_qd  = compute_err_stats(err_qd,  joint_labels, '角速度 qd / (rad/s)');
stats_tau = compute_err_stats(err_tau, joint_labels, '力矩 tau / (N·m)');
fprintf('\n===== 误差统计（实际 - 仿真，左腿 6 关节）=====\n');
print_err_stats(stats_q,   '关节角 q');
print_err_stats(stats_qd,  '角速度 qd');
print_err_stats(stats_tau, '力矩 tau');

%% 1) 关节角 q：期望、实际、仿真
f1 = figure('Name', '关节角对比');
plot_compare_6dof(t, [cmd_l_1, pos_l_1, pos_l_2], 'q', {'期望', '实际', '仿真'});
print(f1, fullfile(out_dir, 'q_compare.png'), '-dpng', '-r300');

%% 2) 关节角误差：实际 - 仿真
f2 = figure('Name', '关节角误差 (实际-仿真)');
plot_compare_6dof(t, [pos_l_1 - pos_l_2], 'q', {'实际-仿真'});
print(f2, fullfile(out_dir, 'q_error.png'), '-dpng', '-r300');

%% 3) 角速度 qd：实际、仿真
f3 = figure('Name', '角速度对比');
plot_compare_6dof(t, [vel_l_1, vel_l_2], 'qd', {'实际', '仿真'});
print(f3, fullfile(out_dir, 'qd_compare.png'), '-dpng', '-r300');

%% 4) 角速度误差：实际 - 仿真
f4 = figure('Name', '角速度误差 (实际-仿真)');
plot_compare_6dof(t, [vel_l_1 - vel_l_2], 'qd', {'实际-仿真'});
print(f4, fullfile(out_dir, 'qd_error.png'), '-dpng', '-r300');

%% 5) 力矩 torque：实际、仿真
f5 = figure('Name', '力矩对比');
plot_compare_6dof(t, [tau_l_1, tau_l_2], 'torque', {'实际', '仿真'});
print(f5, fullfile(out_dir, 'tau_compare.png'), '-dpng', '-r300');

%% 6) 力矩误差：实际 - 仿真
f6 = figure('Name', '力矩误差 (实际-仿真)');
plot_compare_6dof(t, [tau_l_1 - tau_l_2], 'torque', {'实际-仿真'});
print(f6, fullfile(out_dir, 'tau_error.png'), '-dpng', '-r300');

%% 子函数：误差统计与打印
function s = compute_err_stats(err, joint_labels, unit_str)
% err n×6，每列一个关节
% s 含 per_joint (6 个关节的 max_abs, rmse, mae, mean, std)，与 overall（全 6 关节合并）
n_j = size(err, 2);
s.joint_labels = joint_labels;
s.unit = unit_str;
s.max_abs = zeros(1, n_j);
s.rmse    = zeros(1, n_j);
s.mae     = zeros(1, n_j);
s.mean_e  = zeros(1, n_j);
s.std_e   = zeros(1, n_j);
for j = 1:n_j
    e = err(:, j);
    s.max_abs(j) = max(abs(e));
    s.rmse(j)    = sqrt(mean(e.^2));
    s.mae(j)     = mean(abs(e));
    s.mean_e(j)  = mean(e);
    s.std_e(j)   = std(e);
end
% 整体（6 关节一起）
e_all = err(:);
s.overall_max_abs = max(abs(e_all));
s.overall_rmse   = sqrt(mean(e_all.^2));
s.overall_mae    = mean(abs(e_all));
end

function print_err_stats(s, name)
fprintf('\n--- %s (%s) ---\n', name, s.unit);
fprintf('关节\t\tmax|e|\t\tRMSE\t\tMAE\t\tmean(e)\t\tstd(e)\n');
for j = 1:length(s.joint_labels)
    fprintf('%s\t\t%.6g\t\t%.6g\t\t%.6g\t\t%.6g\t\t%.6g\n', ...
        s.joint_labels{j}, s.max_abs(j), s.rmse(j), s.mae(j), s.mean_e(j), s.std_e(j));
end
fprintf('整体\t\t%.6g\t\t%.6g\t\t%.6g\n', s.overall_max_abs, s.overall_rmse, s.overall_mae);
end
