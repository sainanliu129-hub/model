%% compare_sim_vs_real_trajectory  仿真 vs 实测轨迹对比：q / qd / qdd / τ 及误差
%
% 数据文件（左腿）：
%   - 激励轨迹：*_exctra_sim.csv（仿真）、*_exctra_real.csv（实测）
%   - 跑步轨迹：*_sim_run.csv（仿真）、*_real_run.csv（实测）
%
% 流程：激励仿真辨识 → 跑步仿真验证 OK，跑步实测“崩”。本脚本对同一条轨迹类型的
%       仿真与实测做对齐后对比，绘制 q/qd/qdd/τ 曲线及误差，并给出误差统计与原因分析。
%
% 用法：在 Body_GravityPara_Iden 目录下运行本脚本；可改 compare_mode 与 window_opts。

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
repo = fileparts(fileparts(fileparts(app_root)));
if isempty(which('read_leg_joint_csv'))
    addpath(repo);
    addpath(fullfile(repo, 'utility_function'));
end
if isempty(which('continuous_window_id_data'))
    addpath(app_root);
end
if isempty(which('plot_compare_6dof'))
    addpath(fullfile(repo, 'utility_function'));
end

data_dir = fullfile(app_root, 'data', 'excitation');

% 四类文件
file_exctra_sim  = fullfile(data_dir, 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
file_exctra_real = fullfile(data_dir, 'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');
file_sim_run     = fullfile(data_dir, 'PD-M1-v0_multi_joint_20260305-195641_sim_run.csv');
file_real_run    = fullfile(data_dir, 'PD-M1-v0_multi_joint_20260305-195729_real_run.csv');

% 对比模式：'running' = 跑步轨迹仿真 vs 实测；'excitation' = 激励轨迹仿真 vs 实测
compare_mode = 'running';
% 时间窗（与 run_full_dynamics_validation / 调试脚本一致，便于对比）
window_opts = struct('t_start_s', 0, 't_end_s', 10, 'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);   % 不滤波

if strcmp(compare_mode, 'running')
    file_sim  = file_sim_run;
    file_real = file_real_run;
    traj_name = '跑步轨迹';
else
    file_sim  = file_exctra_sim;
    file_real = file_exctra_real;
    traj_name = '激励轨迹';
end

if ~isfile(file_sim)
    error('仿真 CSV 不存在: %s', file_sim);
end
if ~isfile(file_real)
    error('实测 CSV 不存在: %s', file_real);
end

%% 1. 读取并取窗内数据（同一窗 + 同一 qdd 处理）
data_sim  = read_leg_joint_csv(file_sim);
data_real = read_leg_joint_csv(file_real);

t_sim  = data_sim.time(:);
q_sim  = data_sim.pos_leg_l;
qd_sim = data_sim.vel_leg_l;
tau_sim = data_sim.torque_leg_l;

t_real = data_real.time(:);
q_real = data_real.pos_leg_l;
qd_real = data_real.vel_leg_l;
tau_real = data_real.torque_leg_l;

avg_sim  = continuous_window_id_data(t_sim,  q_sim,  qd_sim,  tau_sim,  window_opts);
avg_real = continuous_window_id_data(t_real, q_real, qd_real, tau_real, window_opts);

t_equiv_sim  = avg_sim.t_equiv(:);
t_equiv_real = avg_real.t_equiv(:);
q_sim_w   = avg_sim.q_bar;
qd_sim_w  = avg_sim.qd_bar;
qdd_sim_w = avg_sim.qdd_bar;
tau_sim_w = avg_sim.tau_bar;
q_real_w   = avg_real.q_bar;
qd_real_w  = avg_real.qd_bar;
qdd_real_w = avg_real.qdd_bar;
tau_real_w = avg_real.tau_bar;

M_sim  = size(q_sim_w, 1);
M_real = size(q_real_w, 1);
n = size(q_sim_w, 2);

%% 2. 对齐到公共时间轴（按时间插值，便于逐点误差）
t_min = max(t_equiv_sim(1),  t_equiv_real(1));
t_max = min(t_equiv_sim(end), t_equiv_real(end));
if t_max <= t_min
    error('仿真与实测时间窗无重叠，请检查 t_start_s/t_end_s 或 CSV 时间范围。');
end
N_common = min([M_sim, M_real, 3000]);   % 避免过长
t_common = linspace(t_min, t_max, N_common)';

q_sim_c   = interp1(t_equiv_sim,  q_sim_w,   t_common, 'linear', 'extrap');
qd_sim_c  = interp1(t_equiv_sim,  qd_sim_w,  t_common, 'linear', 'extrap');
qdd_sim_c = interp1(t_equiv_sim,  qdd_sim_w, t_common, 'linear', 'extrap');
tau_sim_c = interp1(t_equiv_sim,  tau_sim_w, t_common, 'linear', 'extrap');
q_real_c   = interp1(t_equiv_real, q_real_w,   t_common, 'linear', 'extrap');
qd_real_c  = interp1(t_equiv_real, qd_real_w,  t_common, 'linear', 'extrap');
qdd_real_c = interp1(t_equiv_real, qdd_real_w, t_common, 'linear', 'extrap');
tau_real_c = interp1(t_equiv_real, tau_real_w, t_common, 'linear', 'extrap');

% 误差：实测 - 仿真（看实测相对仿真的偏差）
err_q   = q_real_c   - q_sim_c;
err_qd  = qd_real_c  - qd_sim_c;
err_qdd = qdd_real_c - qdd_sim_c;
err_tau = tau_real_c  - tau_sim_c;

%% 3. 误差统计（各关节 RMSE、max|e|、MAE）
joint_names = {'leg_11', 'leg_12', 'leg_13', 'leg_14', 'leg_15', 'leg_16'};
rmse_q   = sqrt(mean(err_q.^2,   1));  maxabs_q   = max(abs(err_q),   [], 1);  mae_q   = mean(abs(err_q),   1);
rmse_qd  = sqrt(mean(err_qd.^2,  1));  maxabs_qd  = max(abs(err_qd),  [], 1);  mae_qd  = mean(abs(err_qd),  1);
rmse_qdd = sqrt(mean(err_qdd.^2, 1));  maxabs_qdd = max(abs(err_qdd), [], 1);  mae_qdd = mean(abs(err_qdd), 1);
rmse_tau = sqrt(mean(err_tau.^2, 1)); maxabs_tau = max(abs(err_tau), [], 1);  mae_tau = mean(abs(err_tau), 1);

fprintf('\n===== %s：仿真 vs 实测（实测 - 仿真）=====\n', traj_name);
fprintf('公共时间 [%.2f, %.2f] s，点数 %d\n', t_common(1), t_common(end), N_common);
fprintf('--- 关节角 q (rad) ---\n');  print_row(joint_names, rmse_q,   maxabs_q,   mae_q);
fprintf('--- 角速度 qd (rad/s) ---\n'); print_row(joint_names, rmse_qd,  maxabs_qd,  mae_qd);
fprintf('--- 角加速度 qdd (rad/s^2) ---\n'); print_row(joint_names, rmse_qdd, maxabs_qdd, mae_qdd);
fprintf('--- 力矩 tau (N·m) ---\n'); print_row(joint_names, rmse_tau, maxabs_tau, mae_tau);
fprintf('整体 q   max|e|=%.4f  RMSE=%.4f\n', max(abs(err_q(:))),   sqrt(mean(err_q(:).^2)));
fprintf('整体 qd  max|e|=%.4f  RMSE=%.4f\n', max(abs(err_qd(:))),  sqrt(mean(err_qd(:).^2)));
fprintf('整体 qdd max|e|=%.4f  RMSE=%.4f\n', max(abs(err_qdd(:))), sqrt(mean(err_qdd(:).^2)));
fprintf('整体 tau max|e|=%.4f  RMSE=%.4f\n', max(abs(err_tau(:))), sqrt(mean(err_tau(:).^2)));

%% 4. 绘图：仿真 vs 实测
figure('Name', [traj_name '_q_仿真vs实测']);
plot_compare_6dof(t_common, [q_sim_c, q_real_c], 'q', {'仿真', '实测'});
sgtitle(sprintf('%s 关节角 [%.1f,%.1f]s', traj_name, t_common(1), t_common(end)));

figure('Name', [traj_name '_qd_仿真vs实测']);
plot_compare_6dof(t_common, [qd_sim_c, qd_real_c], 'qd', {'仿真', '实测'});
sgtitle(sprintf('%s 角速度 [%.1f,%.1f]s', traj_name, t_common(1), t_common(end)));

figure('Name', [traj_name '_qdd_仿真vs实测']);
plot_compare_6dof(t_common, [qdd_sim_c, qdd_real_c], 'qdd', {'仿真', '实测'});
sgtitle(sprintf('%s 角加速度 [%.1f,%.1f]s', traj_name, t_common(1), t_common(end)));

figure('Name', [traj_name '_tau_仿真vs实测']);
plot_compare_6dof(t_common, [tau_sim_c, tau_real_c], 'torque', {'仿真', '实测'});
sgtitle(sprintf('%s 力矩 [%.1f,%.1f]s', traj_name, t_common(1), t_common(end)));

%% 5. 误差图
figure('Name', [traj_name '_误差_q']);
for j = 1:n
    subplot(2, 3, j);
    plot(t_common, err_q(:, j)); xlabel('t (s)'); ylabel('\Delta q (rad)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('%s 关节角误差 (实测-仿真)', traj_name));

figure('Name', [traj_name '_误差_qd']);
for j = 1:n
    subplot(2, 3, j);
    plot(t_common, err_qd(:, j)); xlabel('t (s)'); ylabel('\Delta qd (rad/s)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('%s 角速度误差 (实测-仿真)', traj_name));

figure('Name', [traj_name '_误差_qdd']);
for j = 1:n
    subplot(2, 3, j);
    plot(t_common, err_qdd(:, j)); xlabel('t (s)'); ylabel('\Delta qdd (rad/s^2)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('%s 角加速度误差 (实测-仿真)', traj_name));

figure('Name', [traj_name '_误差_tau']);
for j = 1:n
    subplot(2, 3, j);
    plot(t_common, err_tau(:, j)); xlabel('t (s)'); ylabel('\Delta\tau (N\cdot m)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('%s 力矩误差 (实测-仿真)', traj_name));

%% 6. 原因分析摘要（打印到命令窗）
fprintf('\n');
fprintf('========== 仿真 OK、实测“崩”的常见原因 ==========\n');
fprintf('1) 辨识用激励仿真 → X_hat 拟合的是仿真动力学；实测 q/qd 有噪声、τ 来自电流估计，\n');
fprintf('   与仿真不同步/不同尺度，导致 Y_min*X_hat 与 τ_meas 偏差大、FD 的 qdd 量级异常。\n');
fprintf('2) 时间不同步：编码器(q)、驱动器(qd/τ) 可能有时延，同一时间戳下 (q,qd,τ) 非同一时刻。\n');
fprintf('3) qdd 由实测 q 二阶差分+平滑，噪声被放大；若 dt/单位有误，qdd 量级会错（如 10^7 量级）。\n');
fprintf('4) τ_meas 单位/符号/偏置与模型约定不一致（如电流换算系数、某轴取反）。\n');
fprintf('5) 摩擦、柔性、未建模动力学在实机存在，仿真无。\n');
fprintf('建议：先核对 τ 与 (q,qd) 单位与时间对齐；再查 qdd 计算处 dt/time 单位；重力段单独验证。\n');
fprintf('====================================================\n');

%% 子函数
function print_row(joint_names, rmse, maxabs, mae)
fprintf('关节\t\tRMSE\t\tmax|e|\t\tMAE\n');
for j = 1:length(joint_names)
    fprintf('%s\t\t%.4f\t\t%.4f\t\t%.4f\n', joint_names{j}, rmse(j), maxabs(j), mae(j));
end
end
