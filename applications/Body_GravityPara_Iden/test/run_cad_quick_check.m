%% run_cad_quick_check  CAD-only 快速自检（π 顺序 + Y 列范数）
%
% 功能整合自：
%   - run_check_pi_order_vs_urdf
%   - run_check_regressor_column_norms
%
% 1) 打印 π_cad 按 link 的 10 参及列索引范围（便于核对 URDF / ReMatrix 顺序）
% 2) 构造 Y_full，计算各列范数 ||Y(:,j)||，标注对应的 link / 参数名
%
% 默认轨迹来源为 'random'（不依赖任何 CSV / mat），也可切到 'csv' 利用
% data/excitation 下的文件做更真实的激励检查。
%
% 用法：
%   cd 到 Body_GravityPara_Iden/test
%   run_cad_quick_check
%
% 如需使用 CSV：
%   打开本文件，修改下方 trajectory_source = 'csv' 及 csv_file / csv_opts 即可。

clc; clear; close all;

%% 路径与基础配置
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

limb = 'left_leg';
para_order = 1;

% para_order 1: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
% para_order 2: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
param_names_1 = {'m', 'mx', 'my', 'mz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz'};
param_names_2 = {'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz', 'mx', 'my', 'mz', 'm'};
param_names = param_names_1;
if para_order == 2
    param_names = param_names_2;
end

[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
p = numel(pi_cad);
n_links = p / 10;

%% 轨迹配置：random / csv（二选一）
trajectory_source = 'random';  % 'random'（默认）或 'csv'
n_random = 100;
csv_file = fullfile(app_root, 'data', 'excitation', ...
    'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
csv_opts = struct('t_start_s', 2.1, 't_end_s', 4.1, ...
    'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

[q_bar, qd_bar, qdd_bar, M] = local_build_trajectory( ...
    limb, n, trajectory_source, n_random, csv_file, csv_opts);

%% 1) π_cad 与 URDF / para_order 对照（按 link 打印）
fprintf('===== π_cad 与 URDF / para_order 对照 =====\n');
fprintf('  limb = %s, n_links = %d, para_order = %d\n', limb, n_links, para_order);
fprintf('  Y 列序（每 link 10 列）: %s\n', strjoin(param_names, ', '));
fprintf('  对应 get_limb_theta_from_URDF 与 ReMatrix_E1_limb_URDF 一致。\n\n');

fprintf('  link   列索引(j)     ');
for j = 1:10
    fprintf(' %6s', param_names{j});
end
fprintf('\n');
fprintf('  ----  ------------   ');
fprintf(' %6s', repmat('------', 1, 10));
fprintf('\n');

for i = 1:n_links
    idx = (i-1)*10 + (1:10);
    j_range = sprintf('%d..%d', idx(1), idx(10));
    fprintf('  %2d     %-12s  ', i, j_range);
    for j = 1:10
        fprintf(' %6.3e', pi_cad(idx(j)));
    end
    fprintf('\n');
end
fprintf('\n若与 URDF 中 Bodies{1..n} 的 Mass、CenterOfMass、Inertia 不一致，或与 ReMatrix 列序不符，请修 get_limb_theta_from_URDF 或 ReMatrix。\n\n');

%% 2) 构造 Y_full 并计算各列范数
Y_full = [];
for k = 1:M
    Y_one = ReMatrix_E1_limb_URDF(limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, para_order);
    Y_full = [Y_full; Y_one]; %#ok<AGROW>
end

col_norms = sqrt(sum(Y_full.^2, 1));

fprintf('===== Y_full 列范数（轨迹点数 M=%d，para_order=%d） =====\n', M, para_order);
fprintf('  j    link  param   ||Y(:,j)||    备注\n');
for j = 1:p
    link_j = floor((j-1)/10) + 1;
    param_j = mod(j-1, 10) + 1;
    note = '';
    if col_norms(j) < 1e-10
        note = ' 近似为 0，未激励或对应错误';
    end
    fprintf(' %3d   %2d    %-4s   %.4e  %s\n', j, link_j, param_names{param_j}, col_norms(j), note);
end
zero_cols = find(col_norms < 1e-10);
fprintf('\n列范数近似为 0 的列: ');
if isempty(zero_cols)
    fprintf('无\n');
else
    fprintf('%s\n', mat2str(zero_cols));
end

fprintf('\nCAD-only 快速自检结束：若 π 顺序与列范数检查均正常，可继续用 run_check_full_param_consistency_cad_only / run_diagnose_pi_Y_columns 做更深入排查。\n');

%% 局部工具函数
function [q_bar, qd_bar, qdd_bar, M] = local_build_trajectory( ...
    limb, n, trajectory_source, n_random, csv_file, csv_opts)

switch lower(trajectory_source)
    case 'random'
        rng(43);
        q_bar   = (rand(n_random, n) - 0.5) * 1.2;
        qd_bar  = (rand(n_random, n) - 0.5) * 1.0;
        qdd_bar = (rand(n_random, n) - 0.5) * 1.0;
        M = n_random;
        fprintf('轨迹: 随机 %d 点（无文件）\n', M);

    case 'csv'
        if ~isfile(csv_file)
            error('CSV 不存在: %s', csv_file);
        end
        data = read_leg_joint_csv(csv_file);
        t = data.time(:);
        q = data.pos_leg_l;
        qd = data.vel_leg_l;
        tau = data.torque_leg_l;
        avg = continuous_window_id_data(t, q, qd, tau, csv_opts);
        q_bar = avg.q_bar;
        qd_bar = avg.qd_bar;
        qdd_bar = avg.qdd_bar;
        M = size(q_bar, 1);
        fprintf('轨迹: CSV 窗内 %d 点，文件: %s\n', M, csv_file);

    otherwise
        error('未知的 trajectory_source: %s（应为 ''random'' 或 ''csv''）', trajectory_source);
end
end

