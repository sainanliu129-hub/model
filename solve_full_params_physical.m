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
%   .w_m                 默认 20
%   .w_h                 默认 80
%   .w_Idiag             默认 20
%   .w_Ioff              默认 120
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

    opts = set_default(opts, 'diag_dev_frac',    0.8);
    opts = set_default(opts, 'offdiag_dev_abs',  0.05);
    opts = set_default(opts, 'h_dev_abs',        0.10);

    opts = set_default(opts, 'w_m',              20);
    opts = set_default(opts, 'w_h',              80);
    opts = set_default(opts, 'w_Idiag',          20);
    opts = set_default(opts, 'w_Ioff',           120);

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

        % 加权 CAD 正则
        f_reg = 0;
        for ii = 1:n_links
            id = blk_idx(ii);
            b  = pi_vec(id);
            bc = pi_cad(id);

            d_m     = b(1)    - bc(1);
            d_h     = b(2:4)  - bc(2:4);
            d_Idiag = [b(5)-bc(5); b(8)-bc(8); b(10)-bc(10)];
            d_Ioff  = [b(6)-bc(6); b(7)-bc(7); b(9)-bc(9)];

            f_reg = f_reg ...
                + opts.w_m     * (d_m.' * d_m) ...
                + opts.w_h     * (d_h.' * d_h) ...
                + opts.w_Idiag * (d_Idiag.' * d_Idiag) ...
                + opts.w_Ioff  * (d_Ioff.' * d_Ioff);
        end

        f = f_fit + lambda * f_reg;
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

            % ---- 3) 一阶矩相对 CAD 不要漂太远 ----
            dh = h_i - h_cad;
            c_h = [
                dh(1) - opts.h_dev_abs; -dh(1) - opts.h_dev_abs;
                dh(2) - opts.h_dev_abs; -dh(2) - opts.h_dev_abs;
                dh(3) - opts.h_dev_abs; -dh(3) - opts.h_dev_abs];

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

            % ---- 7) 主惯量不要离 CAD 太远 ----
            I_floor = 1e-4;
            dx_lim = opts.diag_dev_frac * max(abs(cad(ii).Ixx), I_floor);
            dy_lim = opts.diag_dev_frac * max(abs(cad(ii).Iyy), I_floor);
            dz_lim = opts.diag_dev_frac * max(abs(cad(ii).Izz), I_floor);

            c_diag_dev = [
                (Ixx - cad(ii).Ixx) - dx_lim;
               -(Ixx - cad(ii).Ixx) - dx_lim;
                (Iyy - cad(ii).Iyy) - dy_lim;
               -(Iyy - cad(ii).Iyy) - dy_lim;
                (Izz - cad(ii).Izz) - dz_lim;
               -(Izz - cad(ii).Izz) - dz_lim];

            % ---- 8) 交叉惯量相对 CAD 不要乱飞 ----
            c_offdiag_dev = [
                (Ixy - cad(ii).Ixy) - opts.offdiag_dev_abs;
               -(Ixy - cad(ii).Ixy) - opts.offdiag_dev_abs;
                (Ixz - cad(ii).Ixz) - opts.offdiag_dev_abs;
               -(Ixz - cad(ii).Ixz) - opts.offdiag_dev_abs;
                (Iyz - cad(ii).Iyz) - opts.offdiag_dev_abs;
               -(Iyz - cad(ii).Iyz) - opts.offdiag_dev_abs];

            c_all = [c_all;
                c_mass_lb; c_mass_ub;
                c_com;
                c_h;
                c_diag_nonneg;
                c_tri;
                c_pd;
                c_offdiag;
                c_diag_dev;
                c_offdiag_dev];
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

    options = optimoptions('fmincon', ...
        'Algorithm', opts.algorithm, ...
        'Display', opts.display, ...
        'MaxIterations', opts.max_iter, ...
        'MaxFunctionEvaluations', opts.MaxFunctionEvaluations, ...
        'SpecifyObjectiveGradient', false, ...
        'SpecifyConstraintGradient', false, ...
        'StepTolerance', 1e-10, ...
        'ConstraintTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-8);

    [pi_phys, fval, exitflag, output] = fmincon( ...
        @obj_fun, pi0, [], [], [], [], [], [], @nonlcon, options);

    % -------------------------
    % 输出信息
    % -------------------------
    info = struct();
    info.fval = fval;
    info.exitflag = exitflag;
    info.output = output;
    info.lambda = lambda;
    info.opts = opts;

    % 额外诊断
    info.fit_res_norm = norm(Y_full*pi_phys - tau_target);
    info.reg_res_norm = norm(pi_phys - pi_cad);

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
