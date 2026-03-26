function [pi_phys, info] = solve_full_params_physical(Y_full, Y_min, beta_hat, pi_cad, lambda, opts)
% solve_full_params_physical
% 带加权 CAD 先验与物理约束的 β -> full π 恢复
%
% 目标：
%   min_pi  ||Y_full*pi - Y_min*beta_hat||^2 + lambda * R(pi, pi_cad)
%
% 其中正则 R 采用“按 link / 按参数类型加权”的形式，避免优化器通过放大
% 一阶矩与交叉惯量来“补偿”回归误差，导致 ID 看着能贴、FD 很差。
%
% -------------------------
% 输入
% -------------------------
% Y_full   : (m x p) full regressor
% Y_min    : (m x r) min regressor
% beta_hat : (r x 1) identified min params
% pi_cad   : (p x 1) CAD / URDF full params
% lambda   : 正则总权重（建议 1 ~ 100；若为空默认 10）
% opts      可选结构体：
%
%   % 基本
%   .max_iter            默认 500
%   .display             默认 'iter'
%   .algorithm           默认 'sqp'
%
%   % 质量约束
%   .m_min_frac          默认 0.5    % m >= 0.5 * m_cad
%   .m_max_frac          默认 1.5    % m <= 1.5 * m_cad
%
%   % CoM 约束（单位：m）
%   .delta_c             默认 0.02   % |c - c_cad|_inf <= delta_c
%
%   % 惯量约束
%   .eps_pd              默认 1e-4   % inertia min eig lower bound
%   .offdiag_max_frac    默认 0.5    % |Ixy| <= frac * sqrt(Ixx*Iyy) 等
%
%   % CAD 偏差软约束（避免跑太远）
%   .diag_dev_frac       默认 0.8    % |Ixx-Ixx_cad| <= 0.8*max(|Ixx_cad|,I_floor)
%   .offdiag_dev_abs     默认 0.05   % |Ixy-Ixy_cad| <= offdiag_dev_abs
%   .h_dev_abs           默认 0.10   % |h-h_cad| <= h_dev_abs (kg*m)
%
%   % 加权正则
%   .w_m                 默认 1
%   .w_h                 默认 5
%   .w_I                 默认 2      % 惯量整体（含对角+交叉）权重
%
%   % 可选：PHYS 阶段轻量质量矩阵健康度惩罚
%   .lambda_M            默认 0      % 建议 1e-3 ~ 1e-2（相对拟合项）
%   .eps_M               默认 1e-6
%   .q_M                 默认 []     % N×n 采样关节角；为空则 J_M 关闭
%   .idx_M               默认 []     % q_M 的子采样索引；空则全用
%   .limb                默认 'left_leg'
%   .para_order          默认 1
%
%   % 可选：H 步串联速度一致项 J_qd（抑制 qdd 尚可但 qd 漂移；H 见 qd_H_steps）
%   .lambda_qd           默认 0
%   .qd_H_steps          默认 1；H>1 时对 qd(k+H) 做 H 步递推预测并与测量比较
%   .qd_err_joint_weights 可选 1×n 向量，对 ||e_j||^2 逐关节加权；空则看 qd_w_j56
%   .qd_w_j56            默认 1；当未提供 qd_err_joint_weights 且 n>=6 时，关节 5/6 使用此权重（再整体归一化使 mean(w)=1）
%   .t_sample            M×1 时间戳（与 q/qd/tau 行对齐）
%   .q_sample, .qd_sample, .tau_sample  M×n 轨迹
%   .idx_qd              采样下标 k（需满足 k+H<=M）；空则复用 idx_M 并自动去掉末 H 点
%
% 输出
% -------------------------
% pi_phys : 恢复得到的 full params
% info    : 诊断信息结构体
%
% 参数顺序（每 link 10 维）：
%   [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]

    if nargin < 5 || isempty(lambda)
        lambda = 10;
    end
    if nargin < 6
        opts = struct();
    end

    % -------------------------
    % 默认参数
    % -------------------------
    opts = set_default(opts, 'max_iter',         500);
    opts = set_default(opts, 'MaxFunctionEvaluations', 5000); % Add default for MaxFunctionEvaluations
    opts = set_default(opts, 'display',          'iter');
    opts = set_default(opts, 'algorithm',        'sqp');

    opts = set_default(opts, 'm_min_frac',       0.5);
    opts = set_default(opts, 'm_max_frac',       1.5);

    opts = set_default(opts, 'delta_c',          0.02);

    opts = set_default(opts, 'eps_pd',           1e-4);
    opts = set_default(opts, 'offdiag_max_frac', 0.5);

    opts = set_default(opts, 'diag_dev_frac',    0.8);   % 兼容旧参数（不再用于主要约束）
    opts = set_default(opts, 'offdiag_dev_abs',  0.05);  % 兼容旧参数（不再用于主要约束）
    opts = set_default(opts, 'h_dev_abs',        0.10);  % 兼容旧参数，等价于 h_box_abs
    opts = set_default(opts, 'h_box_abs',        opts.h_dev_abs);
    opts = set_default(opts, 'I_diag_lim_frac',  [0.5, 1.5]);

    opts = set_default(opts, 'w_m',              1);
    opts = set_default(opts, 'w_h',              5);
    opts = set_default(opts, 'w_I',              []);
    if isempty(opts.w_I)
        if isfield(opts, 'w_Idiag') && ~isempty(opts.w_Idiag)
            opts.w_I = opts.w_Idiag; % 向后兼容旧配置
        else
            opts.w_I = 2;
        end
    end

    opts = set_default(opts, 'lambda_M',         0);
    opts = set_default(opts, 'eps_M',            1e-6);
    opts = set_default(opts, 'q_M',              []);
    opts = set_default(opts, 'idx_M',            []);
    opts = set_default(opts, 'limb',             'left_leg');
    opts = set_default(opts, 'para_order',       1);

    opts = set_default(opts, 'lambda_qd',        0);
    opts = set_default(opts, 't_sample',         []);
    opts = set_default(opts, 'q_sample',         []);
    opts = set_default(opts, 'qd_sample',        []);
    opts = set_default(opts, 'tau_sample',       []);
    opts = set_default(opts, 'idx_qd',           []);
    opts = set_default(opts, 'qd_H_steps',       1);
    opts = set_default(opts, 'qd_w_j56',         1);
    opts = set_default(opts, 'qd_err_joint_weights', []);
    opts = set_default(opts, 'use_best_feasible', true);
    opts = set_default(opts, 'early_stop_feas',   1e-4);

    % -------------------------
    % 输入整理
    % -------------------------
    beta_hat = beta_hat(:);
    pi_cad   = pi_cad(:);

    [m_full, p] = size(Y_full);
    [m_min, r]  = size(Y_min);

    if m_full ~= m_min
        error('solve_full_params_physical: Y_full 与 Y_min 行数不一致。');
    end
    if numel(beta_hat) ~= r
        error('solve_full_params_physical: beta_hat 长度 %d 与 Y_min 列数 %d 不一致。', numel(beta_hat), r);
    end
    if numel(pi_cad) ~= p
        error('solve_full_params_physical: pi_cad 长度 %d 与 Y_full 列数 %d 不一致。', numel(pi_cad), p);
    end
    if mod(p, 10) ~= 0
        error('solve_full_params_physical: p=%d 不是 10 的整数倍。', p);
    end

    n_links = p / 10;
    tau_target = Y_min * beta_hat;

    % forward_dynamics_full：与 collect_mass_matrix_stats 一致，优化过程少报警
    opts_fd_phys = struct('solver', 'Msym', 'regularize_min_eig', 1e-8, 'cond_warn', 1e15);

    % -------------------------
    % 预计算 CAD 信息
    % -------------------------
    cad = repmat(struct( ...
        'm', 0, 'h', zeros(3,1), 'c', zeros(3,1), ...
        'I', zeros(3,3), ...
        'Ixx', 0, 'Ixy', 0, 'Ixz', 0, 'Iyy', 0, 'Iyz', 0, 'Izz', 0), n_links, 1);

    for i = 1:n_links
        idx = blk_idx(i);
        b = pi_cad(idx);

        m_i  = b(1);
        h_i  = b(2:4);
        Ixx  = b(5); Ixy = b(6); Ixz = b(7);
        Iyy  = b(8); Iyz = b(9); Izz = b(10);

        I_i = [Ixx Ixy Ixz;
               Ixy Iyy Iyz;
               Ixz Iyz Izz];

        cad(i).m   = m_i;
        cad(i).h   = h_i;
        cad(i).c   = safe_div(h_i, m_i);
        cad(i).I   = 0.5 * (I_i + I_i.');
        cad(i).Ixx = Ixx;
        cad(i).Ixy = Ixy;
        cad(i).Ixz = Ixz;
        cad(i).Iyy = Iyy;
        cad(i).Iyz = Iyz;
        cad(i).Izz = Izz;
    end

    % -------------------------
    % 初值：CAD
    % -------------------------
    pi0 = pi_cad;

    % -------------------------
    % 目标函数
    % -------------------------
    function f = obj_fun(pi_vec)
        pi_vec = pi_vec(:);

        % 回归等价项
        e_fit = Y_full * pi_vec - tau_target;
        f_fit = e_fit.' * e_fit;

        % 加权 CAD 正则（分块：m / h / I）
        f_reg = 0;
        for ii = 1:n_links
            id = blk_idx(ii);
            b  = pi_vec(id);
            bc = pi_cad(id);

            d_m     = b(1)    - bc(1);
            d_h     = b(2:4)  - bc(2:4);
            d_I     = [b(5)-bc(5); b(6)-bc(6); b(7)-bc(7); ...
                       b(8)-bc(8); b(9)-bc(9); b(10)-bc(10)];

            f_reg = f_reg ...
                + opts.w_m     * (d_m.' * d_m) ...
                + opts.w_h     * (d_h.' * d_h) ...
                + opts.w_I     * (d_I.' * d_I);
        end

        % 轻量 J_M：仅在提供 q_M 且 lambda_M>0 时启用
        J_M = 0;
        if opts.lambda_M > 0 && ~isempty(opts.q_M)
            q_use = opts.q_M;
            if ~isempty(opts.idx_M)
                idx_m = opts.idx_M(:);
                idx_m = idx_m(idx_m >= 1 & idx_m <= size(q_use, 1));
                if ~isempty(idx_m)
                    q_use = q_use(idx_m, :);
                end
            end
            barrier_terms = zeros(size(q_use, 1), 1);
            for kk = 1:size(q_use, 1)
                min_eig_M = get_min_eig_M_from_pi(q_use(kk, :), pi_vec, opts.limb, opts.para_order);
                barrier_terms(kk) = max(0, opts.eps_M - min_eig_M)^2;
            end
            J_M = mean(barrier_terms);
        end

        % H 步串联速度一致 J_qd（见 compute_J_qd_one_step）
        J_qd = 0;
        if opts.lambda_qd > 0
            J_qd = compute_J_qd_one_step(pi_vec, opts, opts_fd_phys);
        end

        f = f_fit + lambda * f_reg + opts.lambda_M * J_M + opts.lambda_qd * J_qd;
    end

    % -------------------------
    % 非线性约束
    % -------------------------
    function [c, ceq] = nonlcon(pi_vec)
        pi_vec = pi_vec(:);
        c_all = [];

        for ii = 1:n_links
            id = blk_idx(ii);
            b  = pi_vec(id);

            m_i  = b(1);
            h_i  = b(2:4);
            Ixx  = b(5); Ixy = b(6); Ixz = b(7);
            Iyy  = b(8); Iyz = b(9); Izz = b(10);

            % CAD
            m_c   = cad(ii).m;
            c_cad = cad(ii).c;
            h_cad = cad(ii).h;
            I_cad = cad(ii).I; %#ok<NASGU>

            % 当前
            c_i = safe_div(h_i, m_i);
            I_i = [Ixx Ixy Ixz;
                   Ixy Iyy Iyz;
                   Ixz Iyz Izz];
            I_i = 0.5 * (I_i + I_i.');

            % ---- 1) 质量上下界 ----
            m_lb = opts.m_min_frac * max(abs(m_c), 1e-3);
            m_ub = opts.m_max_frac * max(abs(m_c), 1e-3);
            c_mass_lb = m_lb - m_i;
            c_mass_ub = m_i - m_ub;

            % ---- 2) CoM 约束 ----
            dc = c_i - c_cad;
            c_com = [
                dc(1) - opts.delta_c;
               -dc(1) - opts.delta_c;
                dc(2) - opts.delta_c;
               -dc(2) - opts.delta_c;
                dc(3) - opts.delta_c;
               -dc(3) - opts.delta_c];

            % ---- 3) 一阶矩 h 显式 box ----
            dh = h_i - h_cad;
            c_h = [
                dh(1) - opts.h_box_abs; -dh(1) - opts.h_box_abs;
                dh(2) - opts.h_box_abs; -dh(2) - opts.h_box_abs;
                dh(3) - opts.h_box_abs; -dh(3) - opts.h_box_abs];

            % ---- 4) 主惯量非负 + 三角不等式 ----
            c_diag_nonneg = [-Ixx; -Iyy; -Izz];
            c_tri = [
                -(Ixx + Iyy - Izz);
                -(Iyy + Izz - Ixx);
                -(Izz + Ixx - Iyy)];

            % ---- 5) 惯量矩阵正定下界 ----
            eigI = eig(I_i);
            c_pd = opts.eps_pd - min(real(eigI));

            % ---- 6) 交叉惯量不应过大 ----
            % |Ixy| <= frac * sqrt(Ixx*Iyy)
            % |Ixz| <= frac * sqrt(Ixx*Izz)
            % |Iyz| <= frac * sqrt(Iyy*Izz)
            sxy = opts.offdiag_max_frac * sqrt(max(Ixx,0) * max(Iyy,0) + 1e-16);
            sxz = opts.offdiag_max_frac * sqrt(max(Ixx,0) * max(Izz,0) + 1e-16);
            syz = opts.offdiag_max_frac * sqrt(max(Iyy,0) * max(Izz,0) + 1e-16);

            c_offdiag = [
                abs(Ixy) - sxy;
                abs(Ixz) - sxz;
                abs(Iyz) - syz];

            % ---- 7) 主惯量显式 box：Idiag ∈ [a,b] * Idiag_cad ----
            % ---- 7.1) 交叉惯量偏差 box（补回：避免交叉项过度改变 M(q) 健康度）----
            dx_off_lim = opts.offdiag_dev_abs;
            c_offdiag_dev = [
                (Ixy - cad(ii).Ixy) - dx_off_lim;
               -(Ixy - cad(ii).Ixy) - dx_off_lim;
                (Ixz - cad(ii).Ixz) - dx_off_lim;
               -(Ixz - cad(ii).Ixz) - dx_off_lim;
                (Iyz - cad(ii).Iyz) - dx_off_lim;
               -(Iyz - cad(ii).Iyz) - dx_off_lim];

            % ---- 7.2) 主惯量相对 CAD 偏差 box（补回：与 Idiag box 叠加，更稳）----
            I_floor = 1e-6;
            dx_lim = opts.diag_dev_frac * max(abs(cad(ii).Ixx), I_floor);
            dy_lim = opts.diag_dev_frac * max(abs(cad(ii).Iyy), I_floor);
            dz_lim = opts.diag_dev_frac * max(abs(cad(ii).Izz), I_floor);
            c_diag_dev = [
                (Ixx - cad(ii).Ixx) - dx_lim;  -(Ixx - cad(ii).Ixx) - dx_lim;
                (Iyy - cad(ii).Iyy) - dy_lim;  -(Iyy - cad(ii).Iyy) - dy_lim;
                (Izz - cad(ii).Izz) - dz_lim;  -(Izz - cad(ii).Izz) - dz_lim];

            % ---- 7.3) Idiag 显式 box：Idiag ∈ [a,b] * Idiag_cad ----
            I_floor = 1e-6;
            aI = opts.I_diag_lim_frac(1);
            bI = opts.I_diag_lim_frac(2);
            Ixx_ref = max(abs(cad(ii).Ixx), I_floor);
            Iyy_ref = max(abs(cad(ii).Iyy), I_floor);
            Izz_ref = max(abs(cad(ii).Izz), I_floor);
            c_Idiag_box = [
                aI * Ixx_ref - Ixx; Ixx - bI * Ixx_ref;
                aI * Iyy_ref - Iyy; Iyy - bI * Iyy_ref;
                aI * Izz_ref - Izz; Izz - bI * Izz_ref];

            c_all = [c_all;
                c_mass_lb; c_mass_ub;
                c_com;
                c_h;
                c_diag_nonneg;
                c_tri;
                c_pd;
                c_offdiag;
                c_offdiag_dev;
                c_diag_dev;
                c_Idiag_box];
        end

        c = c_all;
        ceq = [];
    end

    % -------------------------
    % fmincon 设置
    % -------------------------
    if isempty(which('fmincon'))
        error('solve_full_params_physical 需要 Optimization Toolbox 的 fmincon。');
    end

    % 记录历史最优可行解（避免末次迭代反弹导致返回次优点）
    best_x = [];
    best_feas = inf;
    best_fval = inf;
    best_iter = 0;

    function stop = outfun_track_best(x, optimValues, state) %#ok<INUSD>
        stop = false;
        if ~strcmp(state, 'iter')
            return;
        end
        if isfield(optimValues, 'constrviolation')
            feas = optimValues.constrviolation;
        else
            feas = inf;
        end
        fv = optimValues.fval;
        % 主准则：可行性更好；次准则：在足够可行时选更小目标值
        if (feas < best_feas) || (feas <= 1e-3 && fv < best_fval)
            best_feas = feas;
            best_fval = fv;
            best_x = x(:);
            if isfield(optimValues, 'iteration')
                best_iter = optimValues.iteration;
            end
        end
        % 可选：达到可行阈值后提前停止
        if ~isempty(opts.early_stop_feas) && isfinite(opts.early_stop_feas) && feas <= opts.early_stop_feas
            stop = true;
        end
    end

    options = optimoptions('fmincon', ...
        'Algorithm', opts.algorithm, ...
        'Display', opts.display, ...
        'MaxIterations', opts.max_iter, ...
        'MaxFunctionEvaluations', opts.MaxFunctionEvaluations, ...
        'SpecifyObjectiveGradient', false, ...
        'SpecifyConstraintGradient', false, ...
        'OutputFcn', @outfun_track_best, ...
        'StepTolerance', 1e-10, ...
        'ConstraintTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-8);

    [pi_last, fval_last, exitflag, output] = fmincon( ...
        @obj_fun, pi0, [], [], [], [], [], [], @nonlcon, options);

    % 默认返回历史最优可行点；若未记录到则回退末次点
    if opts.use_best_feasible && ~isempty(best_x)
        pi_phys = best_x;
        fval = obj_fun(pi_phys);
    else
        pi_phys = pi_last;
        fval = fval_last;
    end

    % -------------------------
    % 输出信息
    % -------------------------
    info = struct();
    info.fval = fval;
    info.exitflag = exitflag;
    info.output = output;
    info.lambda = lambda;
    info.opts = opts;
    info.best_feas = best_feas;
    info.best_fval = best_fval;
    info.best_iter = best_iter;
    info.used_best_feasible = opts.use_best_feasible && ~isempty(best_x);
    info.pi_last = pi_last;
    info.fval_last = fval_last;

    % 额外诊断
    info.fit_res_norm = norm(Y_full*pi_phys - tau_target);
    info.reg_res_norm = norm(pi_phys - pi_cad);
    info.J_fit = info.fit_res_norm^2;
    info.J_cad = compute_block_weighted_cad_cost(pi_phys, pi_cad, n_links, opts);
    info.J_M = 0;
    if opts.lambda_M > 0 && ~isempty(opts.q_M)
        q_use = opts.q_M;
        if ~isempty(opts.idx_M)
            idx_m = opts.idx_M(:);
            idx_m = idx_m(idx_m >= 1 & idx_m <= size(q_use, 1));
            if ~isempty(idx_m)
                q_use = q_use(idx_m, :);
            end
        end
        barrier_terms = zeros(size(q_use, 1), 1);
        for kk = 1:size(q_use, 1)
            min_eig_M = get_min_eig_M_from_pi(q_use(kk, :), pi_phys, opts.limb, opts.para_order);
            barrier_terms(kk) = max(0, opts.eps_M - min_eig_M)^2;
        end
        info.J_M = mean(barrier_terms);
    end

    info.J_qd = 0;
    info.qd_err_joint_weights_used = [];
    if opts.lambda_qd > 0
        info.J_qd = compute_J_qd_one_step(pi_phys, opts, opts_fd_phys);
        if isfield(opts, 'qd_sample') && ~isempty(opts.qd_sample)
            info.qd_err_joint_weights_used = get_qd_err_joint_weights_vec(opts, size(opts.qd_sample, 2));
        end
    end

    info.per_link = struct([]);
    for ii = 1:n_links
        id = blk_idx(ii);
        b = pi_phys(id);

        m_i  = b(1);
        h_i  = b(2:4);
        Ixx  = b(5); Ixy = b(6); Ixz = b(7);
        Iyy  = b(8); Iyz = b(9); Izz = b(10);

        I_i = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
        I_i = 0.5 * (I_i + I_i.');
        eigI = eig(I_i);

        info.per_link(ii).m = m_i;
        info.per_link(ii).c = safe_div(h_i, m_i);
        info.per_link(ii).min_eig_I = min(real(eigI));
        info.per_link(ii).cond_I = cond(I_i + 1e-12*eye(3));
        info.per_link(ii).pi_block = b;
    end
end

% ============================================================
% helper
% ============================================================

function idx = blk_idx(i)
    idx = (i-1)*10 + (1:10);
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

function w = get_qd_err_joint_weights_vec(opts, n_j)
% 返回 1×n_j 的逐关节权重（已归一化 mean(w)=1），用于 J_qd 中 sum_j w_j * e_j^2
    w = ones(1, n_j);
    if isfield(opts, 'qd_err_joint_weights') && ~isempty(opts.qd_err_joint_weights)
        w = opts.qd_err_joint_weights(:).';
        if numel(w) ~= n_j
            error('solve_full_params_physical: qd_err_joint_weights 长度=%d，与关节维 n=%d 不一致。', numel(w), n_j);
        end
    elseif isfield(opts, 'qd_w_j56') && ~isempty(opts.qd_w_j56) && opts.qd_w_j56 > 0 && n_j >= 6
        w(5:6) = opts.qd_w_j56;
    end
    wm = mean(w);
    if wm > 0
        w = w / wm;
    end
end

function J_qd = compute_J_qd_one_step(pi_vec, opts, opts_fd)
% H 步串联：qd_pred(k+H)=qd(k)+sum_{j=0}^{H-1} dt*qdd_fd(k+j)
% 与测量 qd(k+H) 的加权平方误差均值；H 由 opts.qd_H_steps 控制（默认 1）
    J_qd = 0;
    if ~isfield(opts, 'lambda_qd') || opts.lambda_qd <= 0
        return;
    end
    if ~isfield(opts, 'q_sample') || isempty(opts.q_sample)
        return;
    end
    q_s = opts.q_sample;
    if ~isfield(opts, 'qd_sample') || isempty(opts.qd_sample) || ...
            ~isfield(opts, 'tau_sample') || isempty(opts.tau_sample) || ...
            ~isfield(opts, 't_sample') || isempty(opts.t_sample)
        return;
    end
    qd_s = opts.qd_sample;
    tau_s = opts.tau_sample;
    t_s = opts.t_sample(:);
    Mnv = size(q_s, 1);
    n_j = size(qd_s, 2);
    w_j = get_qd_err_joint_weights_vec(opts, n_j);
    if size(qd_s, 1) ~= Mnv || size(tau_s, 1) ~= Mnv || numel(t_s) ~= Mnv
        return;
    end
    idxk = [];
    if isfield(opts, 'idx_qd') && ~isempty(opts.idx_qd)
        idxk = opts.idx_qd(:);
    elseif isfield(opts, 'idx_M') && ~isempty(opts.idx_M)
        idxk = opts.idx_M(:);
    end
    H = 1;
    if isfield(opts, 'qd_H_steps') && ~isempty(opts.qd_H_steps) && isfinite(opts.qd_H_steps)
        H = max(1, round(opts.qd_H_steps));
    end
    idxk = idxk(idxk >= 1 & idxk <= Mnv - H);
    if isempty(idxk)
        return;
    end
    acc = 0;
    Ns = numel(idxk);
    dt_med = median(diff(t_s));
    if ~isfinite(dt_med) || dt_med <= 0
        dt_med = 0.002;
    end
    for ii = 1:Ns
        k = idxk(ii);
        pred = qd_s(k, :);
        ok = true;
        for jj = 0:(H - 1)
            kk = k + jj;
            dt_k = t_s(kk + 1) - t_s(kk);
            if ~isfinite(dt_k) || dt_k <= 0
                dt_k = dt_med;
            end
            try
                qdd_fd = forward_dynamics_full(q_s(kk, :), qd_s(kk, :), tau_s(kk, :), pi_vec, ...
                    opts.limb, opts.para_order, opts_fd);
                qdd_fd = qdd_fd(:).';
            catch
                qdd_fd = nan(1, n_j);
            end
            if any(~isfinite(qdd_fd))
                ok = false;
                break;
            end
            pred = pred + dt_k * qdd_fd;
        end
        if ~ok
            acc = acc + 1e6;
        else
            e = qd_s(k + H, :) - pred;
            acc = acc + sum(w_j .* (e.^2));
        end
    end
    J_qd = acc / Ns;
end

function J_cad = compute_block_weighted_cad_cost(pi_vec, pi_cad, n_links, opts)
    d = pi_vec(:) - pi_cad(:);
    J_cad = 0;
    for i = 1:n_links
        i10 = (i-1)*10;
        d_m = d(i10+1);
        d_h = d(i10+2:i10+4);
        d_I = d(i10+5:i10+10);
        J_cad = J_cad ...
            + opts.w_m * (d_m.' * d_m) ...
            + opts.w_h * (d_h.' * d_h) ...
            + opts.w_I * (d_I.' * d_I);
    end
end

function min_eig_M = get_min_eig_M_from_pi(q_row, pi_vec, limb, para_order)
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
