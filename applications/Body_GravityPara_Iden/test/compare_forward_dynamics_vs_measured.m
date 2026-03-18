% compare_forward_dynamics_vs_measured  正动力学：纯仿真积分，用实测力矩驱动，与实测对比
%
% 读取实测数据（q, qd, tau），初值 q_sim(1)=q_data(1)、qd_sim(1)=qd_data(1)；
% 之后每步用 **上一拍仿真状态** q_sim(k-1)、qd_sim(k-1) 与实测力矩 tau(k-1) 做 FD → qdd，
% 再半隐式欧拉积分：qd_sim(k)=qd_sim(k-1)+qdd*dt，q_sim(k)=q_sim(k-1)+qd_sim(k)*dt。
% 即纯 forward simulation（从初值预测整条轨迹），不每步重置到实测，可与 Gazebo 积分器对比。
%
% 整机 18 DoF 与 URDF 一致：左臂 1:3，右臂 4:6，左腿 7:12，右腿 13:18（见 E1_JOINT_ORDER.md）。
% CSV 数据为腿 12 维：列 1:6 左腿、7:12 右腿；臂无数据，填入整机时置 0。
%
% 可选：同时跑“力矩减去膝关节摩擦力后再正动力学”的对比（仅膝关节有摩擦）：
%   compare_forward_dynamics_vs_measured(..., 'compare_with_friction_comp', true)
% 膝关节摩擦模型：静摩擦 τ_s，库仑+粘性 τ_c*sign(qd)+b*qd；参数可覆盖：
%   'knee_friction_tau_s', 3.12, 'knee_friction_tau_c', 2.0267, 'knee_friction_b', 0.4441
%
% 可选：'use_sim_state', true/false
%   false（默认）：FD 与积分用实测上一拍 q(k-1),qd(k-1)，轨迹稳定、不发散，便于出图对比（replay 式）。
%   true：纯正动力学仿真，用 q_sim(k-1),qd_sim(k-1) 推进，可能发散，用于验证积分器一致性。
%
% 可选：将所有生成的图保存为 PNG：'save_png', true；默认保存到当前目录下的 fig_png，也可用 'save_png_dir' 指定路径。
%
% 用法（等价，任选其一）：
%   compare_forward_dynamics_vs_measured('path/to/file.csv')
%   compare_forward_dynamics_vs_measured('csv_file', 'path/to/file.csv')
%   compare_forward_dynamics_vs_measured('path/to/file.csv', 'chain', 'left_leg')
%   csv_file 为必选参数，无默认数据文件。
%
% 同时输出角加速度对比：差分 qdd（实测 qd 中心差分）vs FD qdd（正动力学每步算出的 qdd_sim），并绘图。
% 依赖：read_leg_joint_csv，compute_e1_forward_dynamics_full，get_e1_full_robot_limb_indices

function compare_forward_dynamics_vs_measured(varargin)
param_names = {'csv_file', 'chain', 'max_points', 'integrate_scheme', ...
    'compare_with_friction_comp', 'knee_friction_tau_s', 'knee_friction_tau_c', 'knee_friction_b', 'use_sim_state', 'save_png', 'save_png_dir'};
if nargin >= 1 && ischar(varargin{1}) && ~any(strcmp(varargin{1}, param_names))
    varargin = [{'csv_file', varargin{1}}, varargin(2:end)];
end
p = inputParser;
addParameter(p, 'csv_file', '', @ischar);
addParameter(p, 'chain', 'both_legs', @(x) ismember(x, {'left_leg', 'right_leg', 'both_legs'}));
addParameter(p, 'max_points', Inf, @isnumeric);
addParameter(p, 'integrate_scheme', 'semi_implicit', @(x) ismember(x, {'semi_implicit', 'explicit'}));
addParameter(p, 'compare_with_friction_comp', false, @islogical);
addParameter(p, 'knee_friction_tau_s', 3.12, @isnumeric);    % 静摩擦 N·m
addParameter(p, 'knee_friction_tau_c', 2.0267, @isnumeric); % 库仑摩擦 N·m
addParameter(p, 'knee_friction_b', 0.4441, @isnumeric);     % 粘性 N·m·s/rad
addParameter(p, 'use_sim_state', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=纯仿真(可能发散), false=replay(稳定)
addParameter(p, 'save_png', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x))); % true=将所有生成的图保存为 PNG
addParameter(p, 'save_png_dir', '', @ischar); % PNG 保存目录，空则用当前目录下的 fig_png
parse(p, varargin{:});
opts = p.Results;
% 保证 principle 与路径（直接运行本脚本时需先加路径）
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end
opts.use_sim_state = logical(opts.use_sim_state);
opts.save_png = logical(opts.save_png);
if opts.save_png && isempty(opts.save_png_dir)
    opts.save_png_dir = fullfile(pwd, 'fig_png');
end

%% 1. 读取实测数据（time, pos, vel, torque），csv_file 为必选参数
if isempty(opts.csv_file)
    error('请指定数据文件，例如: compare_forward_dynamics_vs_measured(''csv_file'', ''path/to/file.csv'')');
end
if ~exist(opts.csv_file, 'file')
    error('未找到数据文件: %s', opts.csv_file);
end

fprintf('数据文件: %s\n', opts.csv_file);
data = read_leg_joint_csv(opts.csv_file);
time = data.time;
q    = [data.pos_leg_l, data.pos_leg_r];
qd   = [data.vel_leg_l, data.vel_leg_r];
tau  = [data.torque_leg_l, data.torque_leg_r];

% 清理 NaN/Inf，避免正动力学报错与积分发散
bad_q   = ~isfinite(q);   q(bad_q)   = 0;
bad_qd  = ~isfinite(qd);  qd(bad_qd)  = 0;
bad_tau = ~isfinite(tau); tau(bad_tau) = 0;
if any(bad_q(:)) || any(bad_qd(:)) || any(bad_tau(:))
    fprintf('警告：数据中含非有限值，已置为 0。\n');
end

dt = median(diff(time));
if dt <= 0 || isnan(dt), dt = 0.002; end
n = length(time);

%% 2. 降采样（正动力学仅需 q, qd, tau，不需加速度数据）
if isfinite(opts.max_points) && n > opts.max_points
    idx = round(linspace(1, n, opts.max_points));
    idx = unique(idx);
    time = time(idx); q = q(idx,:); qd = qd(idx,:); tau = tau(idx,:);
    dt = median(diff(time));
    n = length(time);
    fprintf('已降采样至 %d 点\n', n);
end

% 单链路：只取该链对应的列，正动力学按单链计算（'both_legs' 为默认，左右腿都参与）
switch opts.chain
    case 'left_leg'
        col_idx = 1:6;
        limb_name = 'left_leg';
        n_dof = 6;
        joint_names = {'L1','L2','L3','L4','L5','L6'};
    case 'right_leg'
        col_idx = 7:12;
        limb_name = 'right_leg';
        n_dof = 6;
        joint_names = {'R1','R2','R3','R4','R5','R6'};
    otherwise
        col_idx = 1:12;
        limb_name = [];
        n_dof = 12;
        joint_names = {'L1','L2','L3','L4','L5','L6','R1','R2','R3','R4','R5','R6'};
end
q_chain   = q(:, col_idx);
qd_chain  = qd(:, col_idx);
tau_chain = tau(:, col_idx);
fprintf('链路: %s (自由度 %d)\n', opts.chain, n_dof);

%% 3. 正动力学积分（纯仿真）：整机 FD，用上一拍仿真状态推进，仅初值来自实测
% 正确做法：FD 与积分均用 q_sim(k-1), qd_sim(k-1)，力矩用实测 tau(k-1)。
% 半隐式欧拉：qd_k = qd_{k-1} + qdd_{k-1}*dt，q_k = q_{k-1} + qd_k*dt（用更新后的 qd_k）
% 整机与 URDF 一致：左臂 1:3，右臂 4:6，左腿 7:12，右腿 13:18。CSV 列为 1:6 左腿、7:12 右腿。
[idx_left_leg, idx_right_leg, idx_left_arm, idx_right_arm] = get_e1_full_robot_limb_indices();
q_sim  = zeros(n, n_dof);
qd_sim = zeros(n, n_dof);
qdd_sim = zeros(n, n_dof);

q_sim(1,:)  = q_chain(1,:);
qd_sim(1,:) = qd_chain(1,:);

% 若需对比“力矩减膝摩擦”情形，再算一组初值相同的积分
run_friction_comp = opts.compare_with_friction_comp;
if run_friction_comp
    q_sim_nofric  = zeros(n, n_dof);
    qd_sim_nofric = zeros(n, n_dof);
    q_sim_nofric(1,:)  = q_chain(1,:);
    qd_sim_nofric(1,:) = qd_chain(1,:);
    tau_s = opts.knee_friction_tau_s;
    tau_c = opts.knee_friction_tau_c;
    b_vis = opts.knee_friction_b;
    fprintf('将同时积分：τ 实测 与 τ减膝摩擦（τ_s=%.2f, τ_c=%.4f, b=%.4f，仅 L4/R4 膝）\n', tau_s, tau_c, b_vis);
end

use_semi_implicit = strcmpi(opts.integrate_scheme, 'semi_implicit');
if opts.use_sim_state
    fprintf('正在按整机正动力学积分（纯仿真：FD/积分用 q_sim(k-1),qd_sim(k-1)，%d 点, %s）…\n', n, opts.integrate_scheme);
else
    fprintf('正在按整机正动力学积分（replay：FD/积分用实测 q(k-1),qd(k-1)，轨迹稳定，%d 点, %s）…\n', n, opts.integrate_scheme);
end

for k = 2 : n
    % 上一拍状态：use_sim_state 为 true 用仿真，为 false 用实测（稳定、不发散）
    if opts.use_sim_state
        if isempty(limb_name)
            prev_q_l = q_sim(k-1, 1:6);   prev_q_r = q_sim(k-1, 7:12);
            prev_qd_l = qd_sim(k-1, 1:6); prev_qd_r = qd_sim(k-1, 7:12);
        elseif strcmp(limb_name, 'left_leg')
            prev_q_l = q_sim(k-1, 1:6);   prev_q_r = zeros(1,6);
            prev_qd_l = qd_sim(k-1, 1:6); prev_qd_r = zeros(1,6);
        else
            prev_q_l = zeros(1,6); prev_q_r = q_sim(k-1, 1:6);
            prev_qd_l = zeros(1,6); prev_qd_r = qd_sim(k-1, 1:6);
        end
    else
        if isempty(limb_name)
            prev_q_l = q(k-1, 1:6);   prev_q_r = q(k-1, 7:12);
            prev_qd_l = qd(k-1, 1:6); prev_qd_r = qd(k-1, 7:12);
        elseif strcmp(limb_name, 'left_leg')
            prev_q_l = q(k-1, 1:6);   prev_q_r = zeros(1,6);
            prev_qd_l = qd(k-1, 1:6); prev_qd_r = zeros(1,6);
        else
            prev_q_l = zeros(1,6); prev_q_r = q(k-1, 7:12);
            prev_qd_l = zeros(1,6); prev_qd_r = qd(k-1, 7:12);
        end
    end

    q_full  = zeros(18, 1);
    qd_full = zeros(18, 1);
    tau_full = zeros(18, 1);
    q_full(idx_left_arm)  = 0;
    q_full(idx_right_arm) = 0;
    q_full(idx_left_leg)  = prev_q_l';
    q_full(idx_right_leg) = prev_q_r';
    qd_full(idx_left_arm) = 0;
    qd_full(idx_right_arm)= 0;
    qd_full(idx_left_leg) = prev_qd_l';
    qd_full(idx_right_leg)= prev_qd_r';
    tau_full(idx_left_leg)  = tau(k-1, 1:6)';
    tau_full(idx_right_leg) = tau(k-1, 7:12)';
    q_full(~isfinite(q_full)) = 0;
    qd_full(~isfinite(qd_full)) = 0;
    tau_full(~isfinite(tau_full)) = 0;

    qdd_full = compute_e1_forward_dynamics_full(q_full, qd_full, tau_full);
    qdd_left  = qdd_full(idx_left_leg);
    qdd_right = qdd_full(idx_right_leg);

    % 半隐式欧拉：qd_k = qd_{k-1} + qdd*dt，q_k = q_{k-1} + qd_k*dt
    if isempty(limb_name)
        qdd_sim(k, 1:6)  = qdd_left';
        qdd_sim(k, 7:12) = qdd_right';
        qd_sim(k, 1:6)  = prev_qd_l + qdd_sim(k, 1:6)  * dt;
        qd_sim(k, 7:12) = prev_qd_r + qdd_sim(k, 7:12) * dt;
        if use_semi_implicit
            q_sim(k, 1:6)  = prev_q_l + qd_sim(k, 1:6)  * dt;
            q_sim(k, 7:12) = prev_q_r + qd_sim(k, 7:12) * dt;
        else
            q_sim(k, 1:6)  = prev_q_l + prev_qd_l * dt;
            q_sim(k, 7:12) = prev_q_r + prev_qd_r * dt;
        end
    elseif strcmp(limb_name, 'left_leg')
        qdd_sim(k, 1:6) = qdd_left';
        qd_sim(k, 1:6) = prev_qd_l + qdd_sim(k, 1:6) * dt;
        if use_semi_implicit
            q_sim(k, 1:6) = prev_q_l + qd_sim(k, 1:6) * dt;
        else
            q_sim(k, 1:6) = prev_q_l + prev_qd_l * dt;
        end
    else
        qdd_sim(k, 1:6) = qdd_right';
        qd_sim(k, 1:6) = prev_qd_r + qdd_sim(k, 1:6) * dt;
        if use_semi_implicit
            q_sim(k, 1:6) = prev_q_r + qd_sim(k, 1:6) * dt;
        else
            q_sim(k, 1:6) = prev_q_r + prev_qd_r * dt;
        end
    end
    if opts.use_sim_state
        bad = ~isfinite(qd_sim(k,:)); qd_sim(k, bad) = qd_sim(k-1, bad);
        bad = ~isfinite(q_sim(k,:));  q_sim(k, bad)  = q_sim(k-1, bad);
    else
        bad = ~isfinite(qd_sim(k,:)); qd_sim(k, bad) = qd_chain(k, bad);
        bad = ~isfinite(q_sim(k,:));  q_sim(k, bad)  = q_chain(k, bad);
    end
end

%% 3b. 若启用：再用（力矩减膝摩擦）做一遍正动力学积分（上一拍与主循环一致：use_sim_state 控制）
IDX_KNEE = 4;
if run_friction_comp
    tau_s = opts.knee_friction_tau_s;
    tau_c = opts.knee_friction_tau_c;
    b_vis = opts.knee_friction_b;
    if opts.use_sim_state
        fprintf('正在按「τ 减膝摩擦」再做正动力学积分（FD/积分用 q_sim_nofric(k-1)）...\n');
    else
        fprintf('正在按「τ 减膝摩擦」再做正动力学积分（FD/积分用实测 q(k-1),qd(k-1)）...\n');
    end
    for k = 2 : n
        if opts.use_sim_state
            if isempty(limb_name)
                pql = q_sim_nofric(k-1, 1:6); pqr = q_sim_nofric(k-1, 7:12);
                pqdl = qd_sim_nofric(k-1, 1:6); pqdr = qd_sim_nofric(k-1, 7:12);
            elseif strcmp(limb_name, 'left_leg')
                pql = q_sim_nofric(k-1, 1:6); pqr = zeros(1,6); pqdl = qd_sim_nofric(k-1, 1:6); pqdr = zeros(1,6);
            else
                pql = zeros(1,6); pqr = q_sim_nofric(k-1, 1:6); pqdl = zeros(1,6); pqdr = qd_sim_nofric(k-1, 1:6);
            end
        else
            if isempty(limb_name)
                pql = q(k-1, 1:6); pqr = q(k-1, 7:12); pqdl = qd(k-1, 1:6); pqdr = qd(k-1, 7:12);
            elseif strcmp(limb_name, 'left_leg')
                pql = q(k-1, 1:6); pqr = zeros(1,6); pqdl = qd(k-1, 1:6); pqdr = zeros(1,6);
            else
                pql = zeros(1,6); pqr = q(k-1, 7:12); pqdl = zeros(1,6); pqdr = qd(k-1, 7:12);
            end
        end
        q_full  = zeros(18, 1);
        qd_full = zeros(18, 1);
        tau_full = zeros(18, 1);
        q_full(idx_left_arm)  = 0;
        q_full(idx_right_arm) = 0;
        q_full(idx_left_leg)  = pql';
        q_full(idx_right_leg) = pqr';
        qd_full(idx_left_arm)  = 0;
        qd_full(idx_right_arm) = 0;
        qd_full(idx_left_leg) = pqdl';
        qd_full(idx_right_leg)= pqdr';
        tau_left  = tau(k-1, 1:6)';
        tau_right = tau(k-1, 7:12)';
        tau_left(IDX_KNEE)  = tau(k-1, 4)  - knee_friction_torque(pqdl(4), tau(k-1, 4), tau_s, tau_c, b_vis);
        tau_right(IDX_KNEE) = tau(k-1, 10) - knee_friction_torque(pqdr(4), tau(k-1, 10), tau_s, tau_c, b_vis);
        tau_full(idx_left_leg)  = tau_left;
        tau_full(idx_right_leg) = tau_right;
        q_full(~isfinite(q_full)) = 0;
        qd_full(~isfinite(qd_full)) = 0;
        tau_full(~isfinite(tau_full)) = 0;

        qdd_full = compute_e1_forward_dynamics_full(q_full, qd_full, tau_full);
        qdd_left  = qdd_full(idx_left_leg);
        qdd_right = qdd_full(idx_right_leg);

        if isempty(limb_name)
            qd_sim_nofric(k, 1:6)  = pqdl + qdd_left'  * dt;
            qd_sim_nofric(k, 7:12) = pqdr + qdd_right' * dt;
            if use_semi_implicit
                q_sim_nofric(k, 1:6)  = pql + qd_sim_nofric(k, 1:6)  * dt;
                q_sim_nofric(k, 7:12) = pqr + qd_sim_nofric(k, 7:12) * dt;
            else
                q_sim_nofric(k, 1:6)  = pql + pqdl * dt;
                q_sim_nofric(k, 7:12) = pqr + pqdr * dt;
            end
        elseif strcmp(limb_name, 'left_leg')
            qd_sim_nofric(k, 1:6) = pqdl + qdd_left' * dt;
            if use_semi_implicit
                q_sim_nofric(k, 1:6) = pql + qd_sim_nofric(k, 1:6) * dt;
            else
                q_sim_nofric(k, 1:6) = pql + pqdl * dt;
            end
        else
            qd_sim_nofric(k, 1:6) = pqdr + qdd_right' * dt;
            if use_semi_implicit
                q_sim_nofric(k, 1:6) = pqr + qd_sim_nofric(k, 1:6) * dt;
            else
                q_sim_nofric(k, 1:6) = pqr + pqdr * dt;
            end
        end
        if opts.use_sim_state
            bad = ~isfinite(qd_sim_nofric(k,:)); qd_sim_nofric(k, bad) = qd_sim_nofric(k-1, bad);
            bad = ~isfinite(q_sim_nofric(k,:));  q_sim_nofric(k, bad)  = q_sim_nofric(k-1, bad);
        else
            bad = ~isfinite(qd_sim_nofric(k,:)); qd_sim_nofric(k, bad) = qd_chain(k, bad);
            bad = ~isfinite(q_sim_nofric(k,:));  q_sim_nofric(k, bad)  = q_chain(k, bad);
        end
    end
end
% 首拍 qdd_sim 未在循环中赋值，沿用第 2 拍便于与 qdd_diff 对比
if n >= 2
    qdd_sim(1,:) = qdd_sim(2,:);
end

%% 4. 误差与输出（当前链：实测 vs 仿真，q / qd / qdd）
err_q  = q_chain - q_sim;
err_qd = qd_chain - qd_sim;

% 加速度对比：差分 qdd（实测 qd 中心差分）vs FD qdd（正动力学每步算出的 qdd_sim）
qdd_diff = zeros(n, n_dof);
for j = 1:n_dof
    qdd_diff(:, j) = central_diff(qd_chain(:, j), time);
end
err_qdd = qdd_diff - qdd_sim;
rmse_qdd    = sqrt(mean(err_qdd.^2, 1));
max_err_qdd = max(abs(err_qdd), [], 1);

rmse_q      = sqrt(mean(err_q.^2, 1));
rmse_qd     = sqrt(mean(err_qd.^2, 1));
max_err_q   = max(abs(err_q),  [], 1);
max_err_qd  = max(abs(err_qd), [], 1);

fprintf('\n========== 正动力学仿真 vs 实测 [%s] ==========\n', opts.chain);
fprintf('关节角 q (rad)    RMSE:   '); fprintf(' %.4f', rmse_q);     fprintf('\n');
fprintf('关节角 q (rad)    MaxAbs: '); fprintf(' %.4f', max_err_q);  fprintf('\n');
fprintf('角速度 qd (rad/s) RMSE:   '); fprintf(' %.4f', rmse_qd);    fprintf('\n');
fprintf('角速度 qd (rad/s) MaxAbs: '); fprintf(' %.4f', max_err_qd); fprintf('\n');
fprintf('角加速度 qdd (rad/s^2) 差分vsFD RMSE: '); fprintf(' %.4f', rmse_qdd); fprintf('\n');
fprintf('角加速度 qdd (rad/s^2) 差分vsFD MaxAbs: '); fprintf(' %.4f', max_err_qdd); fprintf('\n');

fprintf('\n关节   RMSE(q) rad   Max|q| rad   RMSE(qd) rad/s   Max|qd| rad/s   RMSE(qdd_diff-FD) rad/s^2\n');
for j = 1:n_dof
    fprintf('%s      %10.4f   %10.4f   %10.4f   %10.4f   %10.4f\n', ...
        joint_names{j}, rmse_q(j), max_err_q(j), rmse_qd(j), max_err_qd(j), rmse_qdd(j));
end

if run_friction_comp
    err_q_nofric  = q_chain - q_sim_nofric;
    err_qd_nofric = qd_chain - qd_sim_nofric;
    rmse_q_nofric   = sqrt(mean(err_q_nofric.^2, 1));
    rmse_qd_nofric  = sqrt(mean(err_qd_nofric.^2, 1));
    max_err_q_nofric  = max(abs(err_q_nofric), [], 1);
    max_err_qd_nofric = max(abs(err_qd_nofric), [], 1);
    fprintf('\n========== 正动力学仿真(τ减膝摩擦) vs 实测 [%s] ==========\n', opts.chain);
    fprintf('关节角 q (rad)    RMSE:   '); fprintf(' %.4f', rmse_q_nofric);     fprintf('\n');
    fprintf('关节角 q (rad)    MaxAbs: '); fprintf(' %.4f', max_err_q_nofric);  fprintf('\n');
    fprintf('角速度 qd (rad/s) RMSE:   '); fprintf(' %.4f', rmse_qd_nofric);    fprintf('\n');
    fprintf('角速度 qd (rad/s) MaxAbs: '); fprintf(' %.4f', max_err_qd_nofric); fprintf('\n');
    fprintf('\n关节   RMSE(q) rad   Max|q| rad   RMSE(qd) rad/s   Max|qd| rad/s [τ减膝摩擦]\n');
    for j = 1:n_dof
        fprintf('%s      %10.4f   %10.4f   %10.4f   %10.4f\n', ...
            joint_names{j}, rmse_q_nofric(j), max_err_q_nofric(j), rmse_qd_nofric(j), max_err_qd_nofric(j));
    end
end

%% 5. 绘图：关节角、角速度、角加速度（图名根据当前选择生成）
fig_run_tag = opts.chain;
if opts.use_sim_state
    fig_run_tag = [fig_run_tag '_纯仿真'];
else
    fig_run_tag = [fig_run_tag '_replay'];
end
if run_friction_comp
    fig_run_tag = [fig_run_tag '_含膝摩擦对比'];
end
nr = 2*(n_dof <= 6) + 3*(n_dof > 6);
nc = 3*(n_dof <= 6) + 4*(n_dof > 6);
figure('Name', ['正动力学_关节角_实测vs仿真_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, q_chain(:,j), 'b-', 'DisplayName', '实测'); hold on;
    plot(time, q_sim(:,j), 'r--', 'DisplayName', '仿真 \tau');
    if run_friction_comp
        plot(time, q_sim_nofric(:,j), 'g-.', 'DisplayName', '仿真 \tau减膝摩擦');
    end
    ylabel('q (rad)'); title(joint_names{j}); legend('Location', 'best'); grid on;
end
sgtitle(sprintf('关节角：实测 vs 正动力学仿真 [%s]', opts.chain));
save_fig_png(opts, 'fd_vs_measured_q');

figure('Name', ['正动力学_角速度_实测vs仿真_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, qd_chain(:,j), 'b-', 'DisplayName', '实测'); hold on;
    plot(time, qd_sim(:,j), 'r--', 'DisplayName', '仿真 \tau');
    if run_friction_comp
        plot(time, qd_sim_nofric(:,j), 'g-.', 'DisplayName', '仿真 \tau减膝摩擦');
    end
    ylabel('qd (rad/s)'); title(joint_names{j}); legend('Location', 'best'); grid on;
end
sgtitle(sprintf('角速度：实测 vs 正动力学仿真 [%s]', opts.chain));
save_fig_png(opts, 'fd_vs_measured_qd');

% 误差图：关节角误差 / 角速度误差
figure('Name', ['正动力学_关节角误差_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, err_q(:,j), 'k-');
    ylabel('\Delta q (rad)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('关节角误差：q_{data} - q_{sim} [%s]', opts.chain));
save_fig_png(opts, 'fd_error_q');

figure('Name', ['正动力学_角速度误差_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, err_qd(:,j), 'k-');
    ylabel('\Delta qd (rad/s)'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('角速度误差：qd_{data} - qd_{sim} [%s]', opts.chain));
save_fig_png(opts, 'fd_error_qd');

% 角加速度对比：差分(实测 qd 中心差分) vs FD(正动力学每步 qdd_sim)
figure('Name', ['正动力学_角加速度对比_差分vsFD_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, qdd_diff(:,j), 'b-', 'DisplayName', 'qdd\_diff'); hold on;
    plot(time, qdd_sim(:,j), 'r--', 'DisplayName', 'qdd\_FD');
    ylabel('rad/s^2'); title(sprintf('%s RMSE=%.3f', joint_names{j}, rmse_qdd(j))); legend('Location', 'best'); grid on;
end
sgtitle(sprintf('角加速度：差分(实测 qd 中心差分) vs FD [%s]', opts.chain));
save_fig_png(opts, 'fd_qdd_compare');

figure('Name', ['正动力学_角加速度误差_差分减FD_', fig_run_tag]);
for j = 1:n_dof
    subplot(nr, nc, j);
    plot(time, err_qdd(:,j), 'k-');
    ylabel('rad/s^2'); title(joint_names{j}); grid on;
end
sgtitle(sprintf('角加速度误差：qdd_{diff} - qdd_{FD} [%s]', opts.chain));
save_fig_png(opts, 'fd_qdd_error');

if opts.use_sim_state
    fprintf('\n说明：初值用实测；之后每步用仿真上一拍 q_sim(k-1),qd_sim(k-1) 与 tau(k-1) 做 FD 再积分（纯正动力学仿真，可能发散）。\n');
else
    fprintf('\n说明：初值用实测；之后每步用实测上一拍 q(k-1),qd(k-1) 与 tau(k-1) 做 FD 再积分（replay 式，轨迹稳定）。\n');
end
end

% 绘图与求导已统一到 utility_function：save_fig_png, central_diff, knee_friction_torque
