% example_read_leg_trajectory_and_plot  调用 read_leg_trajectory_from_csv 并绘制 12 关节的位置/速度/加速度
%
% 用法：修改下方 input_csv 为你的文件路径后运行；会生成 500Hz CSV、MAT，并弹出对比图。

clear; clc;

base = fileparts(mfilename('fullpath'));
addpath(fullfile(base, '..'));
addpath(fullfile(base, '..', '..', 'utility_function'));

% 输入 CSV（无时间列则默认按 30Hz 生成时间）
input_csv = '0-260116_run_softly_轻跑步_002.csv';
if ~exist(input_csv, 'file')
    error('文件不存在: %s\n请修改本脚本中的 input_csv 为实际路径。', input_csv);
end

% 输出目录（与 read_leg_trajectory_from_csv 默认一致）
out_dir = get_build_dir('plan');
[~, name, ~] = fileparts(input_csv);
output_csv = fullfile(out_dir, [name '_leg_500Hz.csv']);
output_mat = fullfile(out_dir, [name '_leg_500Hz.mat']);

% 调用：生成 500Hz CSV，并保存 MAT（含插值前后 t / q_leg 及 qd/qdd）
read_leg_trajectory_from_csv(input_csv, ...
    'output_file', output_csv, ...
    'fs_in', 30, ...
    'fs_out', 500, ...
    'save_mat', output_mat);

% 加载 MAT
traj = load(output_mat);
traj = traj.traj;

t_raw = traj.t_raw;
q_raw = traj.q_leg_raw;
t_out = traj.t;
q_out = traj.q_leg;
qd_out = traj.qd;
qdd_out = traj.qdd;
names = traj.joint_names;

% 逆序轨迹（时间从 0 到 T，便于与正序叠画）
t_rev = t_out(end) - t_out(end:-1:1);
q_rev = q_out(end:-1:1, :);
qd_rev = -qd_out(end:-1:1, :);
qdd_rev = qdd_out(end:-1:1, :);

% 保存逆序轨迹：MAT + CSV（与正序同目录、同格式，文件名加 _reverse）
output_csv_rev = fullfile(out_dir, [name '_leg_500Hz_reverse.csv']);
output_mat_rev = fullfile(out_dir, [name '_leg_500Hz_reverse.mat']);
fs_out = 500;
if isfield(traj, 'fs'), fs_out = traj.fs; end
traj_rev = struct('t', t_rev, 'q_leg', q_rev, 'qd', qd_rev, 'qdd', qdd_rev, ...
    'joint_names', {names}, 'fs', fs_out);
save(output_mat_rev, 'traj_rev');
fid = fopen(output_csv_rev, 'w');
if fid ~= -1
    header_cells = [{'time'}, names];
    for c = 1 : numel(header_cells)
        if c < numel(header_cells), fprintf(fid, '%s,', header_cells{c}); else, fprintf(fid, '%s\n', header_cells{c}); end
    end
    for i = 1 : numel(t_rev)
        fprintf(fid, '%.5f', t_rev(i));
        for j = 1 : 12, fprintf(fid, ',%.6f', q_rev(i, j)); end
        fprintf(fid, '\n');
    end
    fclose(fid);
    fprintf('已保存逆序 CSV: %s\n', output_csv_rev);
end
fprintf('已保存逆序 MAT: %s\n', output_mat_rev);

% 由原始数据数值微分得到速度、加速度（仅用于绘图对比）
dt_raw = diff(t_raw);
dt_raw(dt_raw <= 0) = nan;
qd_raw = zeros(size(q_raw));
qdd_raw = zeros(size(q_raw));
for j = 1 : 12
    qd_raw(1:end-1, j) = diff(q_raw(:, j)) ./ dt_raw;
    qd_raw(end, j) = qd_raw(end-1, j);
    qdd_raw(2:end, j) = diff(qd_raw(:, j)) ./ dt_raw;
    qdd_raw(1, j) = qdd_raw(2, j);
end

n_joints = 12;

% 图1：12 关节 × (q, qd, qdd)，插值前(点) vs 插值后正序(线) vs 插值后逆序(虚线)
fig = figure('Name', '腿部轨迹：位置 / 速度 / 加速度（正序+逆序）', 'Position', [40, 40, 1400, 1000]);
for k = 1 : n_joints
    % 位置 q
    subplot(n_joints, 3, (k-1)*3 + 1);
    plot(t_raw, q_raw(:, k), 'o', 'MarkerSize', 2.5, 'Color', [0.2 0.5 0.9]); hold on;
    plot(t_out, q_out(:, k), '-', 'LineWidth', 0.7, 'Color', [0.9 0.3 0.2]);
    plot(t_rev, q_rev(:, k), '--', 'LineWidth', 0.7, 'Color', [0.2 0.7 0.3]);
    hold off; grid on;
    ylabel('q (rad)');
    title(names{k}, 'Interpreter', 'none');
    if k == 1, title([names{k}, '  —  位置'], 'Interpreter', 'none'); end
    if k == n_joints, xlabel('time (s)'); end

    % 速度 qd
    subplot(n_joints, 3, (k-1)*3 + 2);
    plot(t_raw, qd_raw(:, k), 'o', 'MarkerSize', 2.5, 'Color', [0.2 0.5 0.9]); hold on;
    plot(t_out, qd_out(:, k), '-', 'LineWidth', 0.7, 'Color', [0.9 0.3 0.2]);
    plot(t_rev, qd_rev(:, k), '--', 'LineWidth', 0.7, 'Color', [0.2 0.7 0.3]);
    hold off; grid on;
    ylabel('qd (rad/s)');
    if k == 1, title([names{k}, '  —  速度'], 'Interpreter', 'none'); end
    if k == n_joints, xlabel('time (s)'); end

    % 加速度 qdd
    subplot(n_joints, 3, (k-1)*3 + 3);
    plot(t_raw, qdd_raw(:, k), 'o', 'MarkerSize', 2.5, 'Color', [0.2 0.5 0.9]); hold on;
    plot(t_out, qdd_out(:, k), '-', 'LineWidth', 0.7, 'Color', [0.9 0.3 0.2]);
    plot(t_rev, qdd_rev(:, k), '--', 'LineWidth', 0.7, 'Color', [0.2 0.7 0.3]);
    hold off; grid on;
    ylabel('qdd (rad/s^2)');
    if k == 1, title([names{k}, '  —  加速度'], 'Interpreter', 'none'); end
    if k == n_joints, xlabel('time (s)'); end
end
sgtitle(sprintf('输入: %s  |  蓝点=插值前  红实线=正序  绿虚线=逆序', input_csv), 'Interpreter', 'none');
legend({'插值前 (30Hz)', '插值后 正序', '插值后 逆序'}, 'Position', [0.92 0.5 0.06 0.04]);

fprintf('已绘制 12 个关节的位置/速度/加速度（正序+逆序）\n');
fprintf('  正序 CSV: %s  逆序 CSV: %s\n', output_csv, output_csv_rev);
