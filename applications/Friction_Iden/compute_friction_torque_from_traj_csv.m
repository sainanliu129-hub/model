function [time, qd, tau_f] = compute_friction_torque_from_traj_csv(csv_file, tau_c_pos, tau_c_neg, b, varargin)
% compute_friction_torque_from_traj_csv  从轨迹 CSV 第 4 列速度计算摩擦力矩
%
% 适用格式：如 PD-M1-v0_multi_joint_*_traj_1.csv，表头含 time, cmd_*, pos_*, vel_*, torque_* …
%           第 4 列为 vel_leg_l1_joint（rad/s），用于计算该关节的模型摩擦力矩。
%
% 输入:
%   csv_file   - 轨迹 CSV 路径（如 PD-M1-v0_multi_joint_20260206-213622_traj_1.csv）
%   tau_c_pos  - 正向库伦摩擦 N·m
%   tau_c_neg  - 负向库伦摩擦 N·m
%   b          - 粘滞系数 N·m·s/rad
%   varargin   - 可选名值对:
%     'VelocityColumn' - 速度列号，默认 4
%     'VelocityUnit'   - 'rad/s'（默认）或 'rpm'
%
% 输出:
%   time - n×1 时间 (s)
%   qd   - n×1 角速度 (rad/s)
%   tau_f - n×1 摩擦力矩 (N·m)，τ_f = τ_c + b×qd（按正负用 tau_c_pos / tau_c_neg）
%
% 示例:
%   [t, qd, tau_f] = compute_friction_torque_from_traj_csv(...
%       'PD-M1-v0_multi_joint_20260206-213622_traj_1.csv', 1.8, -1.7, 0.05);
%   plot(t, tau_f);

p = inputParser;
addParameter(p, 'VelocityColumn', 4, @isnumeric);
addParameter(p, 'VelocityUnit', 'rad/s', @ischar);
parse(p, varargin{:});
opts = p.Results;

if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

% 读取：带表头，取第 1 列与速度列
try
    data = readmatrix(csv_file, 'NumHeaderLines', 1);
catch
    T = readtable(csv_file);
    time = T.(T.Properties.VariableNames{1});
    col_idx = min(opts.VelocityColumn, width(T));
    qd = table2array(T(:, col_idx));
    if strcmpi(opts.VelocityUnit, 'rpm')
        qd = qd * (2*pi/60);
    end
    rp = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'robot_algorithm');
    if exist(rp, 'dir') && isempty(which('compute_friction_torque'))
        addpath(genpath(rp));
    end
    tau_f = compute_friction_torque(qd(:), 0, b, 'TauCPos', tau_c_pos, 'TauCNeg', tau_c_neg);
    return;
end

time = data(:, 1);
col_idx = min(opts.VelocityColumn, size(data, 2));
qd = data(:, col_idx);
if strcmpi(opts.VelocityUnit, 'rpm')
    qd = qd * (2*pi/60);
end

% 摩擦模型
cur = fileparts(mfilename('fullpath'));
rp = fullfile(cur, '..', '..', 'robot_algorithm');
if exist(rp, 'dir') && isempty(which('compute_friction_torque'))
    addpath(genpath(rp));
end
tau_f = compute_friction_torque(qd(:), 0, b, 'TauCPos', tau_c_pos, 'TauCNeg', tau_c_neg);

end
