%% run_debug_pi_phys_id_fd_plot
%  单独调试：FD 导向 full 参数辨识 π_fd（identify_full_params_for_fd），并绘制正逆动力学图。
%
%  说明：
%   - 本脚本默认只跑 π_fd（42维 FD 导向优化）+ CAD + 最小参数 β 的对比。
%   - 物理约束版 full 参数 π_phys（solve_full_params_physical）目前已关闭（保留代码结构，后续可再打开）。
%
%  流程：
%    1) 从 min_param_id_result.mat 读入 X_hat, index_base, avg_data；
%    2) 用 identify_full_params_for_fd 得到 π_fd；
%    3) 在同一轨迹上算 ID（τ_meas vs Y*π_fd，可选 Y*π_cad、Y_min*X_hat）并绘图；
%    4) 在同一轨迹上算 FD（qdd_id vs FD(π_fd)，可选 FD(CAD)、FD(β)）并绘图。
%
%  依赖：已运行过 run_min_param_id_from_csv 或 run_full_dynamics_validation，
%        生成 applications/Body_GravityPara_Iden/min_param_id_result.mat。
%
%  用法：在 MATLAB 中 cd 到 applications/Body_GravityPara_Iden/test，运行
%        run_debug_pi_phys_id_fd_plot

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 0) 调试开关与“辨识参数”（你主要改这里）
% - 是否保存本次结果（π_fd 以及对比用的输入/选项）
save_result = true;
out_dir = fullfile(app_root, 'build');
out_mat = fullfile(out_dir, 'debug_pi_fd_latest.mat');

% - 是否同时跑 FD 导向 full 参数辨识 π_fd（identify_full_params_for_fd，42维）
run_pi_fd = true;

% - 物理约束版 π_phys（solve_full_params_physical）先不跑
run_pi_phys = false;

% - identify_full_params_for_fd 的参数（你要调的重点在这里）
%   说明：此优化每次迭代会评估多次 FD（有限差分），所以 MaxFunctionEvaluations 很关键。
opts_fd = struct();
opts_fd.w_tau = 3;
opts_fd.w_qdd = 5;
opts_fd.w_cad = 1;   % 增强 CAD 先验，避免惯量跑到非 PSD
opts_fd.w_M   = 1;   % 先关掉 J_M，加速粗跑；需要更稳再改回 10/20
% 约束（先放宽一档，避免 Feasibility 卡死在不可行域）
opts_fd.m_min_frac = 0.5;
opts_fd.m_max_frac = 1.5;
opts_fd.delta_c    = 0.15;  % 若 Feasibility 卡在 CoM，先放宽跑通；收敛后再逐步收紧
opts_fd.eps_I      = 0;     % 先放宽 PSD 下界跑通；收敛后可逐步收紧到 1e-6/1e-4
opts_fd.eps_M      = 1e-6;
opts_fd.algorithm  = 'sqp';
opts_fd.display    = 'iter';
% 收敛相关（通常“不完全收敛”就加这两个）
opts_fd.max_iter   = 50;    % 先粗跑到能动（太大也会很慢）
opts_fd.MaxFunctionEvaluations = 3000;  % 之前常因评估次数不足提前停；先加大保证能跑到收敛趋势
% 抽样点：影响速度/稳定性（点越多越稳但越慢）
opts_fd.n_qdd = 20;    % 初始阶段强烈建议 <= 8
opts_fd.n_reg = 8;    % 初始阶段强烈建议 <= 4（且 w_M=0 时可随意）
% 约束诊断：打印“最严重违规来自哪类约束/哪个 link”
opts_fd.debug_constraints = true;
opts_fd.debug_constraints_every = 50;

% - FD/ID 绘图抽样（仅影响调试速度，不影响 π_phys 的辨识）
M_fd = 500;   % 正动力学对比点数：500/1000/inf

%% 1) 载入最小参数辨识结果
result_mat = fullfile(app_root, 'min_param_id_result.mat');
if ~isfile(result_mat)
    error('未找到 %s。请先运行 run_min_param_id_from_csv 或 run_full_dynamics_validation 生成该文件。', result_mat);
end
ld = load(result_mat);
X_hat      = ld.X_hat(:);
index_base = ld.index_base(:);
avg_data   = ld.avg_data;
fprintf('已加载 min_param_id_result.mat，最小参数维度 p_min = %d\n', numel(X_hat));

%% 2) 构造轨迹与回归矩阵、CAD 全参
limb = 'left_leg';
para_order = 1;
[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
pi_cad = pi_cad(:);

q_bar_id   = avg_data.q_bar;
qd_bar_id  = avg_data.qd_bar;
qdd_bar_id = avg_data.qdd_bar;
tau_bar_id = avg_data.tau_bar;
M = size(q_bar_id, 1);

Y_full_id = ReMatrix_E1_limb_URDF(limb, q_bar_id, qd_bar_id, qdd_bar_id, 1, para_order);
Y_min_id  = Y_full_id(:, index_base);
beta_hat  = X_hat;

%% 2b) （可选）FD 导向 full 参数辨识：π_fd（42维）
pi_fd = [];
info_fd = struct();
if run_pi_fd
    fprintf('\n===== FD 导向 full 参数辨识（identify_full_params_for_fd） =====\n');
    M_id = size(q_bar_id, 1);
    n_qdd = min(opts_fd.n_qdd, M_id);
    n_reg = min(opts_fd.n_reg, M_id);
    opts_fd.idx_qdd = round(linspace(1, M_id, n_qdd)).';
    opts_fd.idx_reg = round(linspace(1, M_id, n_reg)).';
    opts_fd = rmfield(opts_fd, {'n_qdd','n_reg'});

    [pi_fd, info_fd] = identify_full_params_for_fd( ...
        q_bar_id, qd_bar_id, qdd_bar_id, tau_bar_id, ...
        pi_cad, limb, para_order, opts_fd);
    fprintf('  [fd] fmincon exitflag = %d, fval = %.3e\n', info_fd.exitflag, info_fd.fval);
    fprintf('  [fd] rmse_tau = %.4e, rmse_qdd = %.4e\n', info_fd.rmse_tau, info_fd.rmse_qdd);
end

% （可选）物理约束版 full 参数：π_phys（当前关闭）
pi_phys = [];
info_phys = struct();
if run_pi_phys
    fprintf('\n===== 物理约束版 full 参数（solve_full_params_physical） =====\n');
    error('run_pi_phys=true 但本脚本已设置为先不跑 π_phys。需要时我再帮你恢复这段。');
end

if save_result
    if ~isfolder(out_dir), mkdir(out_dir); end
    S = struct();
    S.pi_fd = pi_fd;
    S.info_fd = info_fd;
    S.pi_cad = pi_cad;
    if run_pi_fd, S.opts_fd = opts_fd; end
    S.limb = limb;
    S.para_order = para_order;
    S.index_base = index_base;
    S.X_hat = X_hat;
    S.result_mat = result_mat;
    save(out_mat, '-struct', 'S');
    fprintf('  已保存本次结果: %s\n', out_mat);
end

%% 4) 逆动力学：τ_meas/id 与 Y*π_phys（及可选 Y*π_cad、Y_min*X_hat）
fprintf('\n===== 逆动力学对比（τ_meas vs Y*π_phys） =====\n');
tau_id     = tau_bar_id;   % 参考：预处理/等效轨迹上的力矩
tau_Y_cad  = zeros(M, n);
tau_Y_min  = zeros(M, n);
tau_Y_fd   = zeros(M, n);

for k = 1:M
    qk   = q_bar_id(k, :);
    qdk  = qd_bar_id(k, :);
    qddk = qdd_bar_id(k, :);
    Y_one = ReMatrix_E1_limb_URDF(limb, qk, qdk, qddk, 1, para_order);
    tau_Y_cad(k, :)  = (Y_one * pi_cad).';
    tau_Y_min(k, :)  = (Y_one(:, index_base) * X_hat).';
    if ~isempty(pi_fd)
        tau_Y_fd(k, :) = (Y_one * pi_fd(:)).';
    end
end

rmse_tau_cad  = sqrt(mean((tau_Y_cad  - tau_id).^2, 1));
rmse_tau_min  = sqrt(mean((tau_Y_min  - tau_id).^2, 1));
fprintf('  力矩 RMSE (N·m): τ_meas vs Y*π_cad:  '); fprintf(' %.3f', rmse_tau_cad);  fprintf('\n');
fprintf('  力矩 RMSE (N·m): τ_meas vs Y_min*X_hat: '); fprintf(' %.3f', rmse_tau_min); fprintf('\n');
if ~isempty(pi_fd)
    rmse_tau_fd = sqrt(mean((tau_Y_fd - tau_id).^2, 1));
    fprintf('  力矩 RMSE (N·m): τ_meas vs Y*π_fd:   '); fprintf(' %.3f', rmse_tau_fd); fprintf('\n');
end

% 逆动力学图：实测 vs π_fd（主） + 可选 CAD、最小参数
figure('Name', '逆动力学_π_fd_单独调试');
if ~isempty(pi_fd)
    plot_compare_6dof((1:M)', [tau_id, tau_Y_fd, tau_Y_cad, tau_Y_min], 'torque', ...
        {'\tau_{meas/id}', 'Y\pi_{fd}', 'Y\pi_{cad}', 'Y_{min}X_{hat}'});
else
    plot_compare_6dof((1:M)', [tau_id, tau_Y_cad, tau_Y_min], 'torque', ...
        {'\tau_{meas/id}', 'Y\pi_{cad}', 'Y_{min}X_{hat}'});
end
sgtitle('逆动力学（π_{fd} 单独调试）：实测 vs Y\pi_{fd} vs CAD vs 最小参数');

%% 5) 正动力学：qdd_id 与 FD(π_phys)（及可选 FD(CAD)、FD(β)）
fprintf('\n===== 正动力学对比（qdd_id vs FD(pi_fd)/FD(CAD)/FD(beta)） =====\n');
% 为加快调试可只取前 M_fd 个点
M_fd = min(M, M_fd);   % 可在“调试开关”处设 500/1000/inf
if M_fd < M
    fprintf('  正动力学仅用前 %d 个点（共 %d 点）。\n', M_fd, M);
end

qdd_id      = qdd_bar_id(1:M_fd, :);
qdd_fd_phys = zeros(M_fd, n);
qdd_fd_cad  = zeros(M_fd, n);
qdd_fd_min  = zeros(M_fd, n);
qdd_fd_fd   = zeros(M_fd, n);

for k = 1:M_fd
    qk  = q_bar_id(k, :);
    qdk = qd_bar_id(k, :);
    tauk = tau_bar_id(k, :);
    qdd_fd_cad(k, :)  = forwardDynamics(robot_limb, qk, qdk, tauk).';
    qdd_fd_min(k, :)  = forward_dynamics_min(qk.', qdk.', tauk(:), X_hat, index_base, limb, para_order).';
    if ~isempty(pi_fd)
        qdd_fd_fd(k, :) = forward_dynamics_full(qk, qdk, tauk, pi_fd, limb, para_order).';
    end
end

rmse_qdd_cad  = sqrt(mean((qdd_fd_cad  - qdd_id).^2, 1));
rmse_qdd_min  = sqrt(mean((qdd_fd_min  - qdd_id).^2, 1));
fprintf('  qdd RMSE (rad/s^2): qdd_id vs FD(CAD):     '); fprintf(' %.3f', rmse_qdd_cad);  fprintf('\n');
fprintf('  qdd RMSE (rad/s^2): qdd_id vs FD(beta):    '); fprintf(' %.3f', rmse_qdd_min);  fprintf('\n');
if ~isempty(pi_fd)
    rmse_qdd_fd = sqrt(mean((qdd_fd_fd - qdd_id).^2, 1));
    fprintf('  qdd RMSE (rad/s^2): qdd_id vs FD(pi_fd):   '); fprintf(' %.3f', rmse_qdd_fd); fprintf('\n');
end

% 正动力学图：参考加速度 vs FD(π_fd)（主） + 可选 CAD、β
figure('Name', '正动力学_π_fd_单独调试');
if ~isempty(pi_fd)
    plot_compare_6dof((1:M_fd)', [qdd_id, qdd_fd_fd, qdd_fd_cad], 'qdd', ...
        {'qdd_{id轨迹}', 'FD(\pi_{fd})', 'FD(CAD)'});
    sgtitle('正动力学（π_{fd} 单独调试）：qdd\_id vs FD(π_{fd}) vs FD(CAD)');
else
    plot_compare_6dof((1:M_fd)', [qdd_id, qdd_fd_cad, qdd_fd_min], 'qdd', ...
        {'qdd_{id轨迹}', 'FD(CAD)', 'FD(\beta)'});
    sgtitle('正动力学（π_{fd} 单独调试）：qdd\_id vs FD(CAD) vs FD(β)');
end

fprintf('\n===== π_phys 单独调试与正逆动力学图完成 =====\n');
