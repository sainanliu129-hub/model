% example_read_multi_joint_csv  从多关节 CSV 读取为 q, qd, qdd（左右腿各 6 列）
%
% CSV 表头格式：time, cmd_leg_l1_joint, pos_leg_l1_joint, vel_leg_l1_joint, torque_leg_l1_joint, ...
% 输出：time, q_left, q_right, qd_left, qd_right, qdd_left, qdd_right（各 n×6）
%
% 用法：先执行 addpaths，修改 csv_file 为实际路径后运行。

clear; clc;

base = fileparts(mfilename('fullpath'));
addpath(fullfile(base, '..', '..', 'utility_function'));

csv_file = 'PD-M1-v0_multi_joint_20260206-213622.csv';
if ~exist(csv_file, 'file')
    error('文件不存在: %s\n请修改 csv_file 为实际路径。', csv_file);
end

data = read_leg_joint_csv(csv_file);

time   = data.time;
n      = length(time);
q_left   = data.pos_leg_l;
q_right  = data.pos_leg_r;
qd_left  = data.vel_leg_l;
qd_right = data.vel_leg_r;

dt = median(diff(time));
if isempty(dt) || dt <= 0 || isnan(dt), dt = 0.002; end
qdd_left  = [zeros(1, 6); diff(data.vel_leg_l) / dt];
qdd_right = [zeros(1, 6); diff(data.vel_leg_r) / dt];

fprintf('已读取: %s\n', csv_file);
fprintf('  时间点数 n = %d, dt ≈ %.6f s\n', n, dt);
fprintf('  q_left / q_right: 各 n×6; qd / qdd 同上\n');
