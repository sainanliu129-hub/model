function [pi_fd, info] = identify_full_params_for_fd(q_bar, qd_bar, qdd_bar, tau_bar, pi_cad, limb, para_order, opts)
% identify_full_params_for_fd  FD 导向的 full 参数辨识（同时最小化力矩误差与 qdd 误差）
%
% 目标：得到既能解释力矩、又能稳定做正动力学的 π，满足
%   qdd_FD(π) = M(q,π)^{-1}(τ - h(q,qd,π))
%
% 优化目标（加权和）：
%   J = w_tau*J_tau + w_qdd*J_qdd + w_cad*J_cad + w_M*J_M
%   J_tau  = sum_k ||Y*π - τ_k||^2
%   J_qdd  = sum_k ||qdd_FD(q_k,qd_k,τ_k,π) - qdd_k_ref||^2
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
%     .w_tau, .w_qdd, .w_cad, .w_M   - 权重（仿真建议 1,10,1,10；实机 1,1~3,5,10）
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
%   opts_fd.w_tau = 1; opts_fd.w_qdd = 10; opts_fd.w_cad = 1; opts_fd.w_reg = 10;
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
opts = set_default(opts, 'w_qdd', 10);
opts = set_default(opts, 'w_cad', 1);
opts = set_default(opts, 'w_M', 10);
opts = set_default(opts, 'w_m', 10);
opts = set_default(opts, 'w_h', 30);
opts = set_default(opts, 'w_Idiag', 10);
opts = set_default(opts, 'w_Ioff', 100);
opts = set_default(opts, 'm_min_frac', 0.7);
opts = set_default(opts, 'm_max_frac', 1.3);
opts = set_default(opts, 'delta_c', 0.02);
opts = set_default(opts, 'eps_I', 1e-4);
opts = set_default(opts, 'eps_M', 1e-6);
opts = set_default(opts, 'max_iter', 400);
opts = set_default(opts, 'algorithm', 'sqp');
opts = set_default(opts, 'display', 'iter');
% 约束诊断：定位 Feasibility 卡住原因（默认关闭，避免有限差分刷屏）
opts = set_default(opts, 'debug_constraints', false);
opts = set_default(opts, 'debug_constraints_every', 200);  % 每 N 次非线性约束评估打印一次
% CoM 约束在小质量 link 上会数值敏感（c=h/m）。当 m 很小，改用对一阶矩 h 的约束（或等价放宽）。
opts = set_default(opts, 'com_mass_eps', 1e-3);  % kg，低于该质量阈值认为 CoM 约束不稳定

pi_cad = pi_cad(:);
[M, n] = size(q_bar);
if mod(numel(pi_cad), 10) ~= 0 || numel(pi_cad) ~= 10*n
    error('identify_full_params_for_fd: pi_cad 长度须为 10*n（n=6 腿）。');
end
n_links = n;

% 抽样：J_qdd 与 J_reg 可只用子集
if ~isfield(opts, 'idx_qdd') || isempty(opts.idx_qdd)
    idx_qdd = (1:M)';
else
    idx_qdd = opts.idx_qdd(:);
end
if ~isfield(opts, 'idx_reg') || isempty(opts.idx_reg)
    n_reg = min(20, M);
    idx_reg = round(linspace(1, M, n_reg))';
else
    idx_reg = opts.idx_reg(:);
end

% 预计算 Y_stack：(M*n)×60，tau_vec：(M*n)×1
Y_stack = ReMatrix_E1_limb_URDF(limb, q_bar, qd_bar, qdd_bar, 1, para_order);
tau_vec = tau_bar(:);

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
end

% 目标与约束
obj = @(x) obj_fun(x);
nonlcon = @(x) nonlcon_fun(x);

if isempty(which('fmincon'))
    error('identify_full_params_for_fd 需要 Optimization Toolbox 的 fmincon。');
end
% 无梯度时 fmincon 用有限差分，每次迭代约 (42+1) 次目标调用；单次目标含 idx_qdd 次 FD + idx_reg 次 get_min_eig_M，故总评估次数不宜过大
max_fun_evals = 2000;
if isfield(opts, 'MaxFunctionEvaluations') && ~isempty(opts.MaxFunctionEvaluations)
    max_fun_evals = opts.MaxFunctionEvaluations;
end

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
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-8, ...
    'OptimalityTolerance', 1e-8);

if ~isempty(fd_step)
    options = optimoptions(options, 'FiniteDifferenceStepSize', fd_step);
end

[x_opt, fval, exitflag, output] = fmincon(obj, x0, [], [], [], [], lb, ub, nonlcon, options);

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

qdd_err = zeros(numel(idx_qdd), n);
for ii = 1:numel(idx_qdd)
    k = idx_qdd(ii);
    qdd_fd_k = forward_dynamics_full(q_bar(k,:), qd_bar(k,:), tau_bar(k,:), pi_fd, limb, para_order, opts_fd_call);
    qdd_err(ii, :) = (qdd_fd_k(:) - qdd_bar(k,:)').';
end
info.rmse_qdd = sqrt(mean(qdd_err(:).^2));
info.maxerr_qdd = max(abs(qdd_err(:)));

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
            pi(i10+1)   = x(i7+1);
            pi(i10+2:4) = x(i7+2:4);
            pi(i10+5)   = x(i7+5);
            pi(i10+6:7) = pc(i10+6:7);
            pi(i10+8)   = x(i7+6);
            pi(i10+9)   = pc(i10+9);
            pi(i10+10)  = x(i7+7);
        end
    end

    function x = pi_to_x42(pi, L)
        x = zeros(42, 1);
        for i = 1:L
            i10 = (i-1)*10;
            i7  = (i-1)*7;
            x(i7+1)   = pi(i10+1);
            x(i7+2:4) = pi(i10+2:4);
            x(i7+5)   = pi(i10+5);
            x(i7+6)   = pi(i10+8);
            x(i7+7)   = pi(i10+10);
        end
    end

% ----- 目标函数 -----
    function f = obj_fun(x)
        pi = x42_to_pi(x, pi_cad, n_links);

        % J_tau
        e_tau = Y_stack * pi - tau_vec;
        J_tau = e_tau.' * e_tau;

        % J_qdd
        J_qdd = 0;
        for ii = 1:numel(idx_qdd)
            k = idx_qdd(ii);
            qdd_fd_k = forward_dynamics_full(q_bar(k,:), qd_bar(k,:), tau_bar(k,:), pi, limb, para_order, opts_fd_call);
            e_qdd = qdd_fd_k(:) - qdd_bar(k,:)';
            J_qdd = J_qdd + e_qdd.' * e_qdd;
        end

        % J_cad 加权 (π-π_cad)'*W*(π-π_cad)
        d = pi - pi_cad;
        J_cad = 0;
        for i = 1:n_links
            i10 = (i-1)*10;
            J_cad = J_cad + opts.w_m   * (d(i10+1)^2);
            J_cad = J_cad + opts.w_h   * (d(i10+2)^2 + d(i10+3)^2 + d(i10+4)^2);
            J_cad = J_cad + opts.w_Idiag * (d(i10+5)^2 + d(i10+8)^2 + d(i10+10)^2);
            J_cad = J_cad + opts.w_Ioff  * (d(i10+6)^2 + d(i10+7)^2 + d(i10+9)^2);
        end

% J_M: 惩罚 min(eig(M)) < eps_M
        J_M = 0;
        for ii = 1:numel(idx_reg)
            k = idx_reg(ii);
            min_eig_M = get_min_eig_M(q_bar(k,:), pi, limb, para_order);
            if min_eig_M < opts.eps_M
                J_M = J_M + (opts.eps_M - min_eig_M)^2;
            end
        end

        f = opts.w_tau*J_tau + opts.w_qdd*J_qdd + opts.w_cad*J_cad + opts.w_M*J_M;
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
            c_cad = cad(i).c(:);
            if numel(c_cad) ~= 3
                c_cad = zeros(3,1);
            end
            % 小质量 link：c=h/m 极易放大数值误差，改为约束一阶矩 h 接近 CAD（等价于“对 c 放宽到 m*delta_c”）
            if abs(m_i) < opts.com_mass_eps || abs(cad(i).m) < opts.com_mass_eps
                h_cad = cad(i).m * c_cad;
                % |h - h_cad|_inf <= com_mass_eps * delta_c
                h_lim = opts.com_mass_eps * opts.delta_c;
                dh = (h_i(:) - h_cad(:));
                c_com = [dh(1)-h_lim; -dh(1)-h_lim; ...
                         dh(2)-h_lim; -dh(2)-h_lim; ...
                         dh(3)-h_lim; -dh(3)-h_lim];
            else
                c_i   = safe_div(h_i, m_i);
                c_i   = c_i(:);
                if numel(c_i) ~= 3
                    c_i = zeros(3,1);
                end
                dc1 = c_i(1) - c_cad(1);
                dc2 = c_i(2) - c_cad(2);
                dc3 = c_i(3) - c_cad(3);
                c_com = [dc1-opts.delta_c; -dc1-opts.delta_c; ...
                         dc2-opts.delta_c; -dc2-opts.delta_c; ...
                         dc3-opts.delta_c; -dc3-opts.delta_c];
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
end

% ========== 辅助函数 ==========
function cad = struct_array_cad(pi_cad, n_links)
    cad = repmat(struct('m',0,'c',zeros(3,1)), n_links, 1);
    for i = 1:n_links
        idx = (i-1)*10 + (1:10);
        b = pi_cad(idx);
        m_i = b(1); h_i = b(2:4);
        cad(i).m = m_i;
        cad(i).c = safe_div(h_i, m_i);
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
