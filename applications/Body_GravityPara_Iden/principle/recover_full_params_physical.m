function [pi_rec, info] = recover_full_params_physical(K, beta_hat, pi_cad, opts)
% recover_full_params_physical  带物理一致性约束的 β→π 恢复（每 link 10 参）
%
%   min_pi  ||K*pi - beta_hat||^2 + lambda*||pi - pi_cad||^2
%   s.t.    对每个 link i:
%             m_i >= m_min
%             |c_i - c_i_cad|_inf <= delta_c   (c_i = h_i/m_i)
%             Ixx,Iyy,Izz >= 0  且满足三角不等式
%             I 矩阵的最小特征值 >= eps_pd
%
% 输入:
%   K        - r×p，β = K*π
%   beta_hat - r×1 最小参数估计
%   pi_cad   - p×1 CAD/URDF 全参数
%   opts     - 可选结构体:
%              .lambda    - 正则系数（默认 1e-2）
%              .delta_c   - 质心允许偏移 (m)，默认 0.05
%              .m_min_frac- 质量下界占 CAD 的比例，默认 0.1（即 m_min = 0.1*m_cad）
%              .eps_pd    - 惯量矩阵最小特征值下界，默认 0
%              .max_iter  - fmincon 最大迭代次数（默认 200）
%
% 输出:
%   pi_rec   - p×1 物理一致恢复的全参数
%   info     - 结构体，含 fval、exitflag、output 等

if nargin < 4
    opts = struct();
end
if ~isfield(opts, 'lambda'),     opts.lambda = 1e-2; end
if ~isfield(opts, 'delta_c'),    opts.delta_c = 0.05; end
if ~isfield(opts, 'm_min_frac'), opts.m_min_frac = 0.1; end
if ~isfield(opts, 'eps_pd'),     opts.eps_pd = 0; end
if ~isfield(opts, 'max_iter'),   opts.max_iter = 200; end

beta_hat = beta_hat(:);
pi_cad   = pi_cad(:);
[r, p]   = size(K);
if numel(beta_hat) ~= r
    error('recover_full_params_physical: beta_hat 长度 %d 应与 K 行数 %d 一致。', numel(beta_hat), r);
end
if numel(pi_cad) ~= p
    error('recover_full_params_physical: pi_cad 长度 %d 应与 K 列数 %d 一致。', numel(pi_cad), p);
end

% link 数（每 link 10 参）
if mod(p, 10) ~= 0
    error('recover_full_params_physical: p=%d 不是 10 的整数倍。', p);
end
n_links = p / 10;

% 预计算 CAD 的 mass 与 COM
m_cad   = zeros(n_links, 1);
com_cad = zeros(n_links, 3);
for i = 1:n_links
    idx = (i-1)*10 + (1:10);
    block = pi_cad(idx);
    m_i   = block(1);
    h_i   = block(2:4);
    m_cad(i) = m_i;
    if abs(m_i) > 1e-12
        com_cad(i, :) = (h_i(:) / m_i).';
    else
        com_cad(i, :) = [0 0 0];
    end
end

lambda   = opts.lambda;
delta_c  = opts.delta_c;
eps_pd   = opts.eps_pd;

% 初值：CAD 全参
pi0 = pi_cad;

% 目标函数
    function f = obj_fun(pi_vec)
        pi_vec = pi_vec(:);
        f_beta = norm(K*pi_vec - beta_hat)^2;
        f_reg  = norm(pi_vec - pi_cad)^2;
        f = f_beta + lambda * f_reg;
    end

% 非线性约束
    function [c, ceq] = nonlcon(pi_vec)
        pi_vec = pi_vec(:);
        c_list = [];
        % 针对每个 link 的 10 维块
        for i = 1:n_links
            idx = (i-1)*10 + (1:10);
            block = pi_vec(idx);
            m_i   = block(1);
            h_i   = block(2:4);
            Ixx = block(5);
            Ixy = block(6);
            Ixz = block(7);
            Iyy = block(8);
            Iyz = block(9);
            Izz = block(10);

            % 质量下界：m_i >= m_min
            m_min = opts.m_min_frac * max(abs(m_cad(i)), 1e-3);
            c_mass = m_min - m_i;

            % 质心 box: |c - c_cad|_inf <= delta_c
            if m_i > 1e-6
                c_i = (h_i(:) / m_i).';
            else
                c_i = com_cad(i, :); % 极小质量时，直接贴 CAD
            end
            dc = c_i - com_cad(i, :);
            c_cx1 = dc(1) - delta_c;
            c_cx2 = -dc(1) - delta_c;
            c_cy1 = dc(2) - delta_c;
            c_cy2 = -dc(2) - delta_c;
            c_cz1 = dc(3) - delta_c;
            c_cz2 = -dc(3) - delta_c;

            % 惯量主对角 & 三角不等式
            c_Ixx = -Ixx;
            c_Iyy = -Iyy;
            c_Izz = -Izz;
            c_tri1 = -(Ixx + Iyy - Izz);
            c_tri2 = -(Iyy + Izz - Ixx);
            c_tri3 = -(Izz + Ixx - Iyy);

            % 惯量矩阵最小特征值 >= eps_pd
            I = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
            I_sym = 0.5*(I+I');
            eigI = eig(I_sym);
            min_eig = min(real(eigI));
            c_pd = eps_pd - min_eig;

            c_list = [c_list;
                c_mass;
                c_cx1; c_cx2; c_cy1; c_cy2; c_cz1; c_cz2;
                c_Ixx; c_Iyy; c_Izz;
                c_tri1; c_tri2; c_tri3;
                c_pd];
        end
        c = c_list;
        ceq = [];
    end

% fmincon 设置
if isempty(which('fmincon'))
    error('recover_full_params_physical 需要 Optimization Toolbox 中的 fmincon。');
end

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'MaxIterations', opts.max_iter, ...
    'MaxFunctionEvaluations', max(2000, 20*p), ...
    'SpecifyObjectiveGradient', false, ...
    'SpecifyConstraintGradient', false);

[pi_rec, fval, exitflag, output] = fmincon(@obj_fun, pi0, [], [], [], [], [], [], @nonlcon, options);

info = struct();
info.fval      = fval;
info.exitflag  = exitflag;
info.output    = output;

end

