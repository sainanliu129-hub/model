function [pi_fd, info] = identify_full_params_for_fd(q_bar, qd_bar, qdd_bar, tau_bar, pi_cad, limb, para_order, opts)
fprintf('[identify_full_params_for_fd] 函数开始执行...\n');
% identify_full_params_for_fd  FD 导向的 full 参数辨识（同时最小化力矩误差与 qdd 误差）
%
% 目标：得到既能解释力矩、又能稳定做正动力学的 π，满足
%   qdd_FD(π) = M(q,π)^{-1}(τ - h(q,qd,π))
%
% 优化目标（加权和）：
%   J = w_tau*J_tau + w_qdd*J_qdd + w_traj*J_traj + w_cad*J_cad + w_M*J_M
%   J_tau  = sum_k ||Y*π - τ_k||^2
%   J_qdd  = sum_k ||qdd_FD(q_k,qd_k,τ_k,π) - qdd_k_ref||^2
%   J_traj = 短窗积分轨迹误差（q/qd）
%   J_cad  = (π-π_cad)'*W*(π-π_cad)，W 按参数类型加权
%   J_reg  = sum_s max(0, eps_M - min(eig(M(q_s,π))))^2
%
% 第一版：42 维优化（固定 Ixy,Ixz,Iyz = CAD），变量为每 link [m,mx,my,mz,Ixx,Iyy,Izz]。
%
% 输入：
%   q_bar, qd_bar, qdd_bar, tau_bar - M×n 轨迹（可先抽样 100~300 点）
%   pi_cad   - 60×1 CAD 全参
%   limb     - 'left_leg' | 'right_leg'
%   para_order - 1
%   opts     - 可选：
%     .pi0           - 60×1 初值（缺省用 pi_cad）
%     .fix_offdiag   - true（默认）固定交叉惯量=CAD
%     .w_tau, .w_qdd, .w_traj, .w_cad, .w_M - 权重
%     .traj_Ns, .traj_H, .traj_dt, .alpha_q, .alpha_qd - J_traj 相关参数
%     .idx_traj      - J_traj 起点下标（缺省均匀抽样 traj_Ns 个）
%     .w_m, .w_h, .w_Idiag, .w_Ioff  - J_cad 内权（默认 10,30,10,100）
%     .m_min_frac, .m_max_frac       - 0.7, 1.3
%     .delta_c                       - CoM 约束 0.02 (m)
%     .eps_I                         - 单 link 惯量矩阵 min(eig) 下界 1e-4
%     .eps_M                         - 质量矩阵 min(eig) 惩罚阈值
%     .idx_qdd                       - 用于 J_qdd 的样本下标（缺省全用）
%     .idx_reg                       - 用于 J_reg 的样本下标（缺省均匀抽最多 20 点）
%     .max_iter, .algorithm, .display
%
% 输出：
%   pi_fd - 60×1 辨识得到的 full 参数
%   info  - fval, exitflag, rmse_tau, rmse_qdd, per_link 等
%
% 用法示例（仿真/验证）：
%   opts_fd = struct();
%   opts_fd.w_tau = 1; opts_fd.w_qdd = 10; opts_fd.w_cad = 1; opts_fd.w_M = 10;
%   opts_fd.m_min_frac = 0.7; opts_fd.m_max_frac = 1.3; opts_fd.delta_c = 0.02;
%   % 可选：抽样 100~300 点
%   idx = round(linspace(1, size(q_bar,1), min(200, size(q_bar,1))))';
%   [pi_fd, info] = identify_full_params_for_fd(...
%       q_bar(idx,:), qd_bar(idx,:), qdd_bar(idx,:), tau_bar(idx,:), ...
%       pi_cad, 'left_leg', 1, opts_fd);
%   fprintf('rmse_tau=%.4f, rmse_qdd=%.4f\n', info.rmse_tau, info.rmse_qdd);

if nargin < 8
    opts = struct();
end

% 默认选项
opts = set_default(opts, 'pi0', []);
opts = set_default(opts, 'fix_offdiag', true);
opts = set_default(opts, 'w_tau', 1);
opts = set_default(opts, 'w_qdd', 0);
opts = set_default(opts, 'w_traj', 0);
opts = set_default(opts, 'w_cad', 1);
opts = set_default(opts, 'w_M', 0);
% J_tau 的关节分项加权策略：
%   'global_mse'     : 旧版，一锅端 mean(e(:).^2)
%   'joint_weighted' : 新版，按关节分项并加权求和
opts = set_default(opts, 'tau_loss_mode', 'global_mse');
opts = set_default(opts, 'tau_joint_weight_basis', 'rms'); % 'rms' | 'std' | 'manual'
opts = set_default(opts, 'tau_joint_weight_manual', []);   % 1xn，basis='manual' 时生效
opts = set_default(opts, 'tau_weight_eps', 1e-6);
opts = set_default(opts, 'qdd_fail_penalty', 1e6); % FD 失败/NaN 时的惩罚项（避免优化中断）
opts = set_default(opts, 'traj_Ns', 4);
opts = set_default(opts, 'traj_H', 20);
opts = set_default(opts, 'traj_dt', 0.002);
opts = set_default(opts, 'alpha_q', 1.0);
opts = set_default(opts, 'alpha_qd', 0.3);
opts = set_default(opts, 'w_m', 10);
opts = set_default(opts, 'w_h', 30);
opts = set_default(opts, 'w_Idiag', 10);
opts = set_default(opts, 'w_Ioff', 100);
opts = set_default(opts, 'm_min_frac', 0.7);
opts = set_default(opts, 'm_max_frac', 1.3);
opts = set_default(opts, 'delta_c', 0.02);
opts = set_default(opts, 'eps_I', 1e-4);
opts = set_default(opts, 'eps_M', 1e-6);
% 是否把 CoM 作为 hard constraint 加入 nonlcon
% 当前 baseline 阶段 CoM 硬约束与 CAD/frame 可能不一致，建议先关闭，让一阶矩靠 J_cad 软约束约束。
opts = set_default(opts, 'use_com_constraint', false);
    opts = set_default(opts, 'max_iter', 400);
    opts = set_default(opts, 'MaxFunctionEvaluations', 20000); % Add default for MaxFunctionEvaluations
    opts = set_default(opts, 'algorithm', 'sqp');
    opts = set_default(opts, 'display', 'iter');
% 约束诊断：定位 Feasibility 卡住原因（默认关闭，避免有限差分刷屏）
opts = set_default(opts, 'debug_constraints', false);
opts = set_default(opts, 'debug_constraints_every', 200);  % 每 N 次非线性约束评估打印一次
% 初值/逐 link 的调试打印（baseline 阶段建议关闭）
opts = set_default(opts, 'debug_initial', false);
% CoM 约束在小质量 link 上会数值敏感（c=h/m）。当 m 很小，改用对一阶矩 h 的约束（或等价放宽）。
opts = set_default(opts, 'com_mass_eps', 1e-3);  % kg，低于该质量阈值认为 CoM 约束不稳定
opts = set_default(opts, 'h_lim_abs', 1.0); % 一阶矩 h 的绝对边界 ±1.0 Nm
opts = set_default(opts, 'I_lim_frac', [0.1, 10.0]); % 主惯量相对 CAD 的比例边界 [0.1, 10.0]

pi_cad = pi_cad(:);
[M, n] = size(q_bar);
if mod(numel(pi_cad), 10) ~= 0 || numel(pi_cad) ~= 10*n
    error('identify_full_params_for_fd: pi_cad 长度须为 10*n（n=6 腿）。');
end
n_links = n;

% 抽样：J_qdd 与 J_reg 可只用子集
if ~isfield(opts, 'idx_qdd') || isempty(opts.idx_qdd)
    idx_qdd = zeros(0,1);
else
    idx_qdd = opts.idx_qdd(:);
end
if ~isfield(opts, 'idx_reg') || isempty(opts.idx_reg)
    idx_reg = zeros(0,1);
else
    idx_reg = opts.idx_reg(:);
end

% J_traj 的起点抽样：仅当 w_traj>0 时需要；w_traj=0 时不生成 idx_traj，避免末尾诊断误跑 forward_dynamics
traj_H = max(0, round(opts.traj_H));
max_start = M - traj_H;
if opts.w_traj <= 0
    idx_traj = zeros(0, 1);
elseif traj_H <= 0 || max_start < 1
    idx_traj = zeros(0, 1);
elseif ~isfield(opts, 'idx_traj') || isempty(opts.idx_traj)
    Ns_eff = max(1, round(opts.traj_Ns));
    idx_traj = round(linspace(1, max_start, Ns_eff)).';
else
    idx_traj = opts.idx_traj(:);
    idx_traj = idx_traj(idx_traj >= 1 & idx_traj <= max_start);
end

% 预计算 Y_stack：(M*n)×60，tau_vec：(M*n)×1
Y_stack = ReMatrix_E1_limb_URDF(limb, q_bar, qd_bar, qdd_bar, 1, para_order);
% ReMatrix 行序是 point-major：[点1的n维; 点2的n维; ...]
% 因此 tau 也必须按点堆叠，不能用 tau_bar(:)（那是 joint-major）
tau_vec = reshape(tau_bar', [], 1);

% CAD 按 link 的结构（用于约束与加权）
cad = struct_array_cad(pi_cad, n_links);

% 调用 forward_dynamics_full 时放宽警告阈值，避免优化过程中反复报警（J_M 项已惩罚病态 M）
opts_fd_call = struct('cond_warn', 1e15, 'symmetry_tol_fro', 1e-5);

% 42 维变量：每 link [m,mx,my,mz,Ixx,Iyy,Izz]
x0 = pi_to_x42(pi_cad, n_links);
if ~isempty(opts.pi0)
    x0 = pi_to_x42(opts.pi0(:), n_links);
end

% 边界：仅对 m 设上下界
lb = -inf(42, 1);
ub =  inf(42, 1);
for i = 1:n_links
    m_c = cad(i).m;
    lb((i-1)*7+1) = opts.m_min_frac * max(abs(m_c), 1e-3);
    ub((i-1)*7+1) = opts.m_max_frac * max(abs(m_c), 1e-3);

    % 一阶矩 h = m*c 边界
    for j = 1:3
        h_cad_val = cad(i).m * cad(i).c(j);
        lower_bound_h = h_cad_val - opts.h_lim_abs;
        upper_bound_h = h_cad_val + opts.h_lim_abs;
        
        % 确保下界不大于上界
        if lower_bound_h > upper_bound_h
            [lower_bound_h, upper_bound_h] = deal(upper_bound_h, lower_bound_h);
        end
        
        lb((i-1)*7+1+j) = lower_bound_h;
        ub((i-1)*7+1+j) = upper_bound_h;
    end

    % 主惯量 Ixx, Iyy, Izz 边界
    Ixx_c = cad(i).I_diag(1); Iyy_c = cad(i).I_diag(2); Izz_c = cad(i).I_diag(3);
    % 确保 CAD 值非负，避免乘以负数产生错误边界
    Ixx_c = max(Ixx_c, 1e-6); Iyy_c = max(Iyy_c, 1e-6); Izz_c = max(Izz_c, 1e-6);

    lb((i-1)*7+5) = opts.I_lim_frac(1) * Ixx_c; % Ixx
    ub((i-1)*7+5) = opts.I_lim_frac(2) * Ixx_c;

    lb((i-1)*7+6) = opts.I_lim_frac(1) * Iyy_c; % Iyy (x(i7+6) 是 Iyy)
    ub((i-1)*7+6) = opts.I_lim_frac(2) * Iyy_c;

    lb((i-1)*7+7) = opts.I_lim_frac(1) * Izz_c; % Izz (x(i7+7) 是 Izz)
    ub((i-1)*7+7) = opts.I_lim_frac(2) * Izz_c;
end


% 目标与约束
obj = @(x) obj_fun(x);
nonlcon = @(x) nonlcon_fun(x);

if isempty(which('fmincon'))
    error('identify_full_params_for_fd 需要 Optimization Toolbox 的 fmincon。');
end
% 无梯度时 fmincon 用有限差分，每次迭代约 (42+1) 次目标调用；单次目标含 idx_qdd 次 FD + idx_reg 次 get_min_eig_M，故总评估次数不宜过大
max_fun_evals = opts.MaxFunctionEvaluations;

% 有限差分设置（可选）：用 forward 可将每次梯度评估的函数调用数减半（相对 central）
fd_type = 'forward';
if isfield(opts, 'FiniteDifferenceType') && ~isempty(opts.FiniteDifferenceType)
    fd_type = opts.FiniteDifferenceType;
end
fd_step = [];
if isfield(opts, 'FiniteDifferenceStepSize') && ~isempty(opts.FiniteDifferenceStepSize)
    fd_step = opts.FiniteDifferenceStepSize;
end

options = optimoptions('fmincon', ...
    'Algorithm', opts.algorithm, ...
    'Display', opts.display, ...
    'MaxIterations', opts.max_iter, ...
    'MaxFunctionEvaluations', max_fun_evals, ...
    'SpecifyObjectiveGradient', false, ...
    'SpecifyConstraintGradient', false, ...
    'FiniteDifferenceType', fd_type, ...
    'StepTolerance', 1e-8, ...
    'ConstraintTolerance', 1e-4, ...
    'OptimalityTolerance', 1e-6, ...
    'OutputFcn', @output_fcn);
if ~isempty(fd_step)
    options = optimoptions(options, 'FiniteDifferenceStepSize', fd_step);
end

fprintf('[identify_full_params_for_fd] 准备调用 fmincon...\n');

        x0 = min(max(x0, lb), ub); % 手动将 x0 投影到边界内
        if opts.debug_initial
            % 调试信息：检查 x0 是否在边界内，并在 fmincon 前分解约束/目标
            violates_lb = x0 < lb;
            violates_ub = x0 > ub;
            if any(violates_lb)
                fprintf('  WARNING: x0 violates lower bound at indices: %s\n', mat2str(find(violates_lb)));
                fprintf('           x0 values: %s\n', mat2str(x0(violates_lb)'));
                fprintf('           lb values: %s\n', mat2str(lb(violates_lb)'));
            end
            if any(violates_ub)
                fprintf('  WARNING: x0 violates upper bound at indices: %s\n', mat2str(find(violates_ub)));
                fprintf('           x0 values: %s\n', mat2str(x0(violates_ub)'));
                fprintf('           ub values: %s\n', mat2str(ub(violates_ub)'));
            end
            fprintf('  Min x0: %.4f, Max x0: %.4f\n', min(x0), max(x0));
            fprintf('  Min lb: %.4f, Max lb: %.4f\n', min(lb), max(lb));
            fprintf('  Min ub: %.4f, Max ub: %.4f\n', min(ub), max(ub));

            fprintf('isempty(opts.pi0) = %d\n', isempty(opts.pi0));
            i = 3; % 第3个link
            i10 = (i-1)*10;
            i7  = (i-1)*7;
            fprintf('--- link %d check ---\n', i);
            fprintf('pi_cad block = %s\n', mat2str(pi_cad(i10+(1:10))', 6));
            fprintf('x0 block     = %s\n', mat2str(x0(i7+(1:7))', 6));
            fprintf('cad.m        = %.6f\n', cad(i).m);
            fprintf('cad.c        = %s\n', mat2str(cad(i).c', 6));
            fprintf('cad.m*cad.c  = %s\n', mat2str((cad(i).m * cad(i).c(:))', 6));
            fprintf('lb block     = %s\n', mat2str(lb(i7+(1:7))', 6));
            fprintf('ub block     = %s\n', mat2str(ub(i7+(1:7))', 6));

            f0 = obj(x0);
            [c0, ~] = nonlcon(x0);
            fprintf('Initial objective f0 = %.6e\n', f0);
            fprintf('Initial feasibility max(c+) = %.6e\n', max([0; c0]));
            % 手动打印每个 link 的约束分解
            pi0_dbg = x42_to_pi(x0, pi_cad, n_links);
            for i = 1:n_links
                i10 = (i-1)*10;
                m_i = pi0_dbg(i10+1);
                h_i = pi0_dbg(i10+2:4);
                Ixx = pi0_dbg(i10+5); Ixy = pi0_dbg(i10+6); Ixz = pi0_dbg(i10+7);
                Iyy = pi0_dbg(i10+8); Iyz = pi0_dbg(i10+9); Izz = pi0_dbg(i10+10);
                c_cad_current = cad(i).c(:);
                if numel(c_cad_current) ~= 3
                    c_cad_current = zeros(3,1);
                end
                if abs(m_i) < opts.com_mass_eps || abs(cad(i).m) < opts.com_mass_eps
                    h_cad = cad(i).m * c_cad_current;
                    h_lim = opts.com_mass_eps * opts.delta_c;
                    dh = (h_i(:) - h_cad(:));
                    c_com = [dh(1)-h_lim; -dh(1)-h_lim; ...
                             dh(2)-h_lim; -dh(2)-h_lim; ...
                             dh(3)-h_lim; -dh(3)-h_lim];
                else
                    c_i = safe_div(h_i, m_i);
                    if numel(c_i) ~= 3
                        c_i = zeros(3,1);
                    else
                        c_i = c_i(:);
                    end
                    dc1 = c_i(1) - c_cad_current(1);
                    dc2 = c_i(2) - c_cad_current(2);
                    dc3 = c_i(3) - c_cad_current(3);
                    c_com = [dc1-opts.delta_c; -dc1-opts.delta_c; ...
                             dc2-opts.delta_c; -dc2-opts.delta_c; ...
                             dc3-opts.delta_c; -dc3-opts.delta_c];
                end
                c_diag = [-Ixx; -Iyy; -Izz];
                c_tri  = [-(Ixx+Iyy-Izz); -(Iyy+Izz-Ixx); -(Izz+Ixx-Iyy)];
                I_i = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
                I_i = 0.5 * (I_i + I_i.');
                eigI = eig(I_i);
                c_pd = opts.eps_I - min(real(eigI));
                fprintf(['link %d: max(c_com)=%.6e, max(c_diag)=%.6e, ' ...
                         'max(c_tri)=%.6e, c_pd=%.6e\n'], ...
                         i, max([0;c_com]), max([0;c_diag]), max([0;c_tri]), max(0,c_pd));
            end
        end

        [x_opt, fval, exitflag, output] = fmincon(obj, x0, [], [], [], [], lb, ub, nonlcon, options);

fprintf('[identify_full_params_for_fd] fmincon 调用结束，exitflag = %d\n', exitflag);
pi_fd = x42_to_pi(x_opt, pi_cad, n_links);

% 诊断
info = struct();
info.fval = fval;
info.exitflag = exitflag;
info.output = output;
info.opts = opts;

tau_pred = Y_stack * pi_fd;
info.rmse_tau = sqrt(mean((tau_pred - tau_vec).^2));
info.maxerr_tau = max(abs(tau_pred - tau_vec));

% 目标函数分解（用于判断“力矩是否确实被优化了”）
tau_pred_mat = reshape(tau_pred, [n, M]).';
info.J_tau = compute_tau_loss(tau_pred_mat, tau_bar);
info.J_cad = 0;
if opts.w_cad > 0
    d = pi_fd - pi_cad;
    weighted_param_err_sq = zeros(10*n_links, 1);
    param_idx_count = 0;
    for i = 1:n_links
        i10 = (i-1)*10;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_m   * (d(i10+1)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+2)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+3)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+4)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+5)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+6)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+7)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+8)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+9)^2); param_idx_count = param_idx_count + 1;
        weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+10)^2); param_idx_count = param_idx_count + 1;
    end
    info.J_cad = mean(weighted_param_err_sq(1:param_idx_count));
end

qdd_err = zeros(numel(idx_qdd), n);
for ii = 1:numel(idx_qdd)
    k = idx_qdd(ii);
    qdd_fd_k = forward_dynamics_full(q_bar(k,:), qd_bar(k,:), tau_bar(k,:), pi_fd, limb, para_order, opts_fd_call);
    qdd_err(ii, :) = (qdd_fd_k(:) - qdd_bar(k,:)').';
end
info.rmse_qdd = sqrt(mean(qdd_err(:).^2));
info.maxerr_qdd = max(abs(qdd_err(:)));

% 轨迹短窗误差诊断：与 J_traj 一致，仅在 w_traj>0 且 idx_traj 非空时执行（避免 w_traj=0 仍递推导致 NaN）
if opts.w_traj > 0 && ~isempty(idx_traj)
    traj_q_err2_sum = 0;
    traj_qd_err2_sum = 0;
    traj_cnt = 0;
    for ii = 1:numel(idx_traj)
        s = idx_traj(ii);
        q_sim = q_bar(s, :);
        qd_sim = qd_bar(s, :);
        for h = 1:traj_H
            k_tau = s + h - 1;
            k_ref = s + h;
            if k_tau > M || k_ref > M
                break;
            end
            if any(~isfinite(q_sim(:))) || any(~isfinite(qd_sim(:)))
                break;
            end
            try
                qdd_sim = forward_dynamics_full(q_sim, qd_sim, tau_bar(k_tau,:), pi_fd, limb, para_order, opts_fd_call);
            catch
                break;
            end
            if any(~isfinite(qdd_sim(:)))
                break;
            end
            qd_sim = qd_sim + opts.traj_dt * qdd_sim(:).';
            q_sim  = q_sim  + opts.traj_dt * qd_sim;

            dq = q_sim - q_bar(k_ref, :);
            dqd = qd_sim - qd_bar(k_ref, :);
            traj_q_err2_sum = traj_q_err2_sum + sum(dq.^2);
            traj_qd_err2_sum = traj_qd_err2_sum + sum(dqd.^2);
            traj_cnt = traj_cnt + n;
        end
    end
    if traj_cnt > 0
        info.rmse_traj_q = sqrt(traj_q_err2_sum / traj_cnt);
        info.rmse_traj_qd = sqrt(traj_qd_err2_sum / traj_cnt);
    else
        info.rmse_traj_q = 0;
        info.rmse_traj_qd = 0;
    end
else
    info.rmse_traj_q = 0;
    info.rmse_traj_qd = 0;
end

info.per_link = struct([]);
for i = 1:n_links
    idx = (i-1)*10 + (1:10);
    b = pi_fd(idx);
    m_i = b(1); h_i = b(2:4);
    Ixx = b(5); Ixy = b(6); Ixz = b(7); Iyy = b(8); Iyz = b(9); Izz = b(10);
    I_i = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
    I_i = 0.5 * (I_i + I_i.');
    eigI = eig(I_i);
    info.per_link(i).m = m_i;
    info.per_link(i).c = safe_div(h_i, m_i);
    info.per_link(i).min_eig_I = min(real(eigI));
    info.per_link(i).cond_I = cond(I_i + 1e-12*eye(3));
    info.per_link(i).pi_block = b;
end

% ----- 嵌套：x(42) <-> pi(60) -----
    function pi = x42_to_pi(x, pc, L)
        pi = zeros(60, 1);
        for i = 1:L
            i10 = (i-1)*10;
            i7  = (i-1)*7;
            pi(i10+1) = x(i7+1);
            pi((i10+2):(i10+4)) = x((i7+2):(i7+4)); % mx,my,mz
            pi(i10+5) = x(i7+5);
            pi((i10+6):(i10+7)) = pc((i10+6):(i10+7)); % Ixy, Ixz
            pi(i10+8) = x(i7+6);
            pi(i10+9) = pc(i10+9); % Iyz
            pi(i10+10) = x(i7+7);
        end
    end

    function x = pi_to_x42(pi, L)
        x = zeros(42, 1);
        for i = 1:L
            i10 = (i-1)*10;
            i7  = (i-1)*7;
        x(i7+1)   = pi(i10+1);       % m
        x(i7+2)   = pi(i10+2);       % mx
        x(i7+3)   = pi(i10+3);       % my
        x(i7+4)   = pi(i10+4);       % mz
        x(i7+5)   = pi(i10+5);       % Ixx
        x(i7+6)   = pi(i10+8);       % Iyy
        x(i7+7)   = pi(i10+10);      % Izz
        end
    end

% ----- 目标函数 -----
    function f = obj_fun(x)
        pi = x42_to_pi(x, pi_cad, n_links);

        % 生成 J_traj 的完整索引
        traj_fd_idx = [];
        if opts.w_traj > 0
            for it = 1:numel(idx_traj)
                s = idx_traj(it);
                traj_fd_idx = [traj_fd_idx; (s : s + traj_H - 1)'];
            end
            % 确保 traj_fd_idx 中的所有索引都在数据范围内
            traj_fd_idx = traj_fd_idx(traj_fd_idx >= 1 & traj_fd_idx <= M);
        end

        need_qdd_calc = (opts.w_qdd > 0 && ~isempty(idx_qdd));
        need_M_calc   = (opts.w_M > 0 && ~isempty(idx_reg));

        all_idx_for_fd_qdd_reg = [];
        if need_qdd_calc
            all_idx_for_fd_qdd_reg = [all_idx_for_fd_qdd_reg; idx_qdd];
        end
        if need_M_calc
            all_idx_for_fd_qdd_reg = [all_idx_for_fd_qdd_reg; idx_reg];
        end
        all_idx_for_fd_qdd_reg = unique(all_idx_for_fd_qdd_reg);
        
        qdd_fd_precalc = [];
        min_eig_M_precalc = [];
        map_qdd_reg_lookup = [];

        if ~isempty(all_idx_for_fd_qdd_reg)
            q_bar_qdd_reg = q_bar(all_idx_for_fd_qdd_reg, :);
            qd_bar_qdd_reg = qd_bar(all_idx_for_fd_qdd_reg, :);
            tau_bar_qdd_reg = tau_bar(all_idx_for_fd_qdd_reg, :);
            
            map_qdd_reg_lookup = zeros(M, 1);
            map_qdd_reg_lookup(all_idx_for_fd_qdd_reg) = 1:numel(all_idx_for_fd_qdd_reg);
            
            qdd_fd_precalc = zeros(numel(all_idx_for_fd_qdd_reg), n);
            for ii = 1:numel(all_idx_for_fd_qdd_reg)
                k_sub = ii; 
                try
                    qdd_try = forward_dynamics_full(q_bar_qdd_reg(k_sub,:), qd_bar_qdd_reg(k_sub,:), tau_bar_qdd_reg(k_sub,:), pi, limb, para_order, opts_fd_call);
                    if any(~isfinite(qdd_try(:)))
                        qdd_fd_precalc(k_sub, :) = nan(1, n);
                    else
                        qdd_fd_precalc(k_sub, :) = qdd_try(:).';
                    end
                catch
                    qdd_fd_precalc(k_sub, :) = nan(1, n);
                end
            end
            
            if need_M_calc
                min_eig_M_precalc = zeros(numel(idx_reg), 1);
                for ii = 1:numel(idx_reg)
                    k_orig = idx_reg(ii);
                    k_sub_idx = map_qdd_reg_lookup(k_orig);
                    min_eig_M_precalc(ii) = get_min_eig_M(q_bar_qdd_reg(k_sub_idx,:), pi, limb, para_order);
                end
            end
        end


        % J_tau：支持“全局MSE”与“按关节分项加权”两种模式
        tau_pred_vec = Y_stack * pi;
        tau_pred_mat = reshape(tau_pred_vec, [n, M]).';
        J_tau = compute_tau_loss(tau_pred_mat, tau_bar);

        % J_qdd
        J_qdd = 0;
        if need_qdd_calc
            qdd_err_sq_sum = 0;
            qdd_cnt = 0;
            for ii = 1:numel(idx_qdd)
                k = idx_qdd(ii);
                k_sub_idx = map_qdd_reg_lookup(k);
                qdd_fd_k = qdd_fd_precalc(k_sub_idx, :); % 使用预计算结果
                if any(~isfinite(qdd_fd_k(:)))
                    qdd_err_sq_sum = qdd_err_sq_sum + opts.qdd_fail_penalty;
                    qdd_cnt = qdd_cnt + 1;
                else
                    e_qdd = qdd_fd_k(:) - qdd_bar(k,:)';
                    qdd_err_sq_sum = qdd_err_sq_sum + sum(e_qdd.^2);
                    qdd_cnt = qdd_cnt + numel(e_qdd);
                end
            end
            J_qdd = qdd_err_sq_sum / max(qdd_cnt,1);
        end

        % J_traj：短窗积分轨迹误差 (真正的多步递推，并调整为均方误差形式)
        J_traj = 0;
        if opts.w_traj > 0
            J_q = 0;
            J_qd = 0;
            traj_point_count = 0;
            for it = 1:numel(idx_traj)
                s = idx_traj(it);
                q_sim = q_bar(s, :);    % 从真实数据开始
                qd_sim = qd_bar(s, :);  % 从真实数据开始
                for h = 1:traj_H
                    k_tau = s + h - 1; 
                    k_ref = s + h;     
    
                    if k_tau > M || k_ref > M 
                        break; 
                    end
    
                    qdd_sim = forward_dynamics_full(q_sim, qd_sim, tau_bar(k_tau,:), pi, limb, para_order, opts_fd_call);
                    qd_sim = qd_sim + opts.traj_dt * qdd_sim(:).';
                    q_sim  = q_sim  + opts.traj_dt * qd_sim;
    
                    dq = q_sim - q_bar(k_ref, :);
                    dqd = qd_sim - qd_bar(k_ref, :);
                    J_q = J_q + sum(dq.^2);
                    J_qd = J_qd + sum(dqd.^2);
                    traj_point_count = traj_point_count + n;
                end
            end
            
            if traj_point_count > 0
                J_q = J_q / traj_point_count;
                J_qd = J_qd / traj_point_count;
            else
                J_q = 0;
                J_qd = 0;
            end
            J_traj = opts.alpha_q * J_q + opts.alpha_qd * J_qd;
        end

        % J_cad 加权 (π-π_cad)'*W*(π-π_cad) - 调整为均方误差形式
        J_cad = 0;
        if opts.w_cad > 0
            d = pi - pi_cad;
            weighted_param_err_sq = zeros(10*n_links, 1);
            param_idx_count = 0;
            for i = 1:n_links
                i10 = (i-1)*10;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_m   * (d(i10+1)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+2)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+3)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_h   * (d(i10+4)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+5)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+6)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+7)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+8)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Ioff  * (d(i10+9)^2); param_idx_count = param_idx_count + 1;
                weighted_param_err_sq(param_idx_count + 1) = opts.w_Idiag * (d(i10+10)^2); param_idx_count = param_idx_count + 1;
            end
            J_cad = mean(weighted_param_err_sq(1:param_idx_count));
        end

        % J_M: 惩罚 min(eig(M)) < eps_M - 调整为均方误差形式
        J_M = 0;
        if need_M_calc
            barrier_terms = zeros(numel(idx_reg), 1);
            for ii = 1:numel(idx_reg)
                min_eig_M = min_eig_M_precalc(ii); 
                if min_eig_M < opts.eps_M
                    barrier_terms(ii) = (opts.eps_M - min_eig_M)^2;
                end
            end
            J_M = mean(barrier_terms);
        end

        f = opts.w_tau*J_tau + opts.w_qdd*J_qdd + opts.w_traj*J_traj + opts.w_cad*J_cad + opts.w_M*J_M;
    end

% ----- 非线性约束（每 link：CoM、I 正定+三角） -----
    function [c, ceq] = nonlcon_fun(x)
        persistent nlc_calls
        if isempty(nlc_calls), nlc_calls = 0; end
        nlc_calls = nlc_calls + 1;

        pi = x42_to_pi(x, pi_cad, n_links);
        c_list = [];

        % 约束违规诊断（只在 debug_constraints=true 时启用）
        worst = struct('val', -inf, 'type', '', 'link', nan);
        worst_per_type = struct('com', -inf, 'diag', -inf, 'tri', -inf, 'pd', -inf);
        worst_link_per_type = struct('com', nan, 'diag', nan, 'tri', nan, 'pd', nan);

        for i = 1:n_links
            i10 = (i-1)*10;
            m_i = pi(i10+1);
            h_i = pi(i10+2:4);
            Ixx = pi(i10+5); Ixy = pi(i10+6); Ixz = pi(i10+7);
            Iyy = pi(i10+8); Iyz = pi(i10+9); Izz = pi(i10+10);

            % CoM 约束：按分量单独计算，避免任何维度/行列混淆
            % 确保 c_cad 是 3x1 向量，统一处理
            c_com = zeros(6,1); % 始终保留 CoM 约束维度，避免 fmincon 非线性约束维度变化
            if opts.use_com_constraint
                c_cad_current = cad(i).c(:);
                if numel(c_cad_current) ~= 3
                    c_cad_current = zeros(3,1);
                end

                if abs(m_i) < opts.com_mass_eps || abs(cad(i).m) < opts.com_mass_eps
                    h_cad = cad(i).m * c_cad_current;
                    % |h - h_cad|_inf <= com_mass_eps * delta_c
                    h_lim = opts.com_mass_eps * opts.delta_c;
                    dh = (h_i(:) - h_cad(:));
                    c_com = [dh(1)-h_lim; -dh(1)-h_lim; ...
                             dh(2)-h_lim; -dh(2)-h_lim; ...
                             dh(3)-h_lim; -dh(3)-h_lim];
                else
                    c_i = safe_div(h_i, m_i);
                    % 确保 c_i 是 3x1 向量
                    if numel(c_i) ~= 3
                        c_i = zeros(3,1);
                    else
                        c_i = c_i(:); % 确保是列向量
                    end

                    dc1 = c_i(1) - c_cad_current(1);
                    dc2 = c_i(2) - c_cad_current(2);
                    dc3 = c_i(3) - c_cad_current(3);
                    c_com = [dc1-opts.delta_c; -dc1-opts.delta_c; ...
                             dc2-opts.delta_c; -dc2-opts.delta_c; ...
                             dc3-opts.delta_c; -dc3-opts.delta_c];
                end
            end

            c_diag = [-Ixx; -Iyy; -Izz];
            c_tri = [-(Ixx+Iyy-Izz); -(Iyy+Izz-Ixx); -(Izz+Ixx-Iyy)];

            I_i = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
            I_i = 0.5 * (I_i + I_i.');
            eigI = eig(I_i);
            c_pd = opts.eps_I - min(real(eigI));

            c_list = [c_list; c_com; c_diag; c_tri; c_pd];

            if opts.debug_constraints
                % 只关心 c>0 的违规量
                v_com  = max(c_com);
                v_diag = max(c_diag);
                v_tri  = max(c_tri);
                v_pd   = c_pd;

                if v_com > worst_per_type.com
                    worst_per_type.com = v_com; worst_link_per_type.com = i;
                end
                if v_diag > worst_per_type.diag
                    worst_per_type.diag = v_diag; worst_link_per_type.diag = i;
                end
                if v_tri > worst_per_type.tri
                    worst_per_type.tri = v_tri; worst_link_per_type.tri = i;
                end
                if v_pd > worst_per_type.pd
                    worst_per_type.pd = v_pd; worst_link_per_type.pd = i;
                end

                % 全局最差
                [v_max_i, idx_max] = max([v_com, v_diag, v_tri, v_pd]);
                if v_max_i > worst.val
                    worst.val = v_max_i;
                    worst.link = i;
                    if idx_max == 1, worst.type = 'CoM(|c-c_cad|_inf<=delta_c)';
                    elseif idx_max == 2, worst.type = 'I_diag_nonneg(Ixx/Iyy/Izz>=0)';
                    elseif idx_max == 3, worst.type = 'triangle(Ixx+Iyy>=Izz,etc)';
                    else, worst.type = 'PSD(minEig(I)>=eps_I)';
                    end
                end
            end
        end
        c = c_list;
        ceq = [];

        if opts.debug_constraints && (mod(nlc_calls, max(1, opts.debug_constraints_every)) == 0)
            feas = max(0, max(c));
            fprintf('  [nlc] call=%d, feasibility=max(c+)=%.3e, worst=%s @link%d (%.3e)\n', ...
                nlc_calls, feas, worst.type, worst.link, worst.val);
            fprintf('        per-type: com=%.3e(link%d), diag=%.3e(link%d), tri=%.3e(link%d), pd=%.3e(link%d)\n', ...
                worst_per_type.com,  worst_link_per_type.com, ...
                worst_per_type.diag, worst_link_per_type.diag, ...
                worst_per_type.tri,  worst_link_per_type.tri, ...
                worst_per_type.pd,   worst_link_per_type.pd);
        end
    end

    function J_tau = compute_tau_loss(tau_pred_mat, tau_meas_mat)
        if strcmpi(opts.tau_loss_mode, 'global_mse')
            e = tau_pred_mat - tau_meas_mat;
            J_tau = mean(e(:).^2);
            return;
        end

        % 默认：joint_weighted
        e = tau_pred_mat - tau_meas_mat;
        J_tau = 0;
        for j = 1:n
            e_j = e(:, j);
            t_j = tau_meas_mat(:, j);
            switch lower(opts.tau_joint_weight_basis)
                case 'std'
                    w_j = std(t_j, 1) + opts.tau_weight_eps; % 与 mean(e_j.^2)更匹配，使用总体标准差
                case 'manual'
                    if isempty(opts.tau_joint_weight_manual) || numel(opts.tau_joint_weight_manual) ~= n
                        error('identify_full_params_for_fd: tau_joint_weight_manual 需为 1xn 或 nx1。');
                    end
                    w_j = opts.tau_joint_weight_manual(j);
                otherwise % 'rms'
                    w_j = sqrt(mean(t_j.^2)) + opts.tau_weight_eps;
            end
            J_tau = J_tau + w_j * mean(e_j.^2);
        end
    end

    function stop = output_fcn(x, optimValues, state)
        stop = false;
        switch state
            case 'init'
                fprintf('fmincon output function initialized.\n');
            case 'iter'
                fprintf('fmincon iteration %d: Fval = %.3e, Feasibility = %.3e\n', ...
                    optimValues.iteration, optimValues.fval, optimValues.constrviolation);
            case 'done'
                fprintf('fmincon output function finalized.\n');
        end
    end
end

% ========== 辅助函数 ==========
function cad = struct_array_cad(pi_cad, n_links)
    cad = repmat(struct('m',0,'c',zeros(3,1), 'I_diag',zeros(3,1)), n_links, 1);
    for i = 1:n_links
        idx = (i-1)*10 + (1:10);
        b = pi_cad(idx);
        m_i = b(1); h_i = b(2:4);
        cad(i).m = m_i;
        cad(i).c = safe_div(h_i, m_i);
        cad(i).I_diag = [b(5); b(8); b(10)]; % Ixx, Iyy, Izz
    end
end

function min_eig_M = get_min_eig_M(q_row, pi_vec, limb, para_order)
    n = numel(q_row);
    q_row = q_row(:).';
    qd_zero = zeros(1, n);
    qdd_zero = zeros(1, n);
    pi_vec = pi_vec(:);
    ID = @(q, qd, qdd) ReMatrix_E1_limb_URDF(limb, q, qd, qdd, 1, para_order) * pi_vec;
    tau_g = ID(q_row, qd_zero, qdd_zero);
    M = zeros(n, n);
    for j = 1:n
        e_j = zeros(1, n); e_j(j) = 1;
        M(:, j) = ID(q_row, qd_zero, e_j) - tau_g;
    end
    Msym = 0.5 * (M + M.');
    min_eig_M = min(real(eig(Msym)));
end

function v = safe_div(h, m)
    if abs(m) < 1e-12
        v = zeros(3,1);
    else
        v = h(:) / m;
    end
end

function s = set_default(s, name, val)
    if ~isfield(s, name)
        s.(name) = val;
    end
end
