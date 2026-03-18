function [qdd, diagOut] = forward_dynamics_full(q, qd, tau, pi_vec, limb, para_order, opts)
% forward_dynamics_full  基于全参数 π 的正动力学（τ = Y*π，用回归矩阵构造 M、h 后求 qdd）
%
% τ = M(q,π)*qdd + h(q,qd,π) => qdd = M \ (tau - h).
% M、h 通过 ReMatrix 在 (q,qd,qdd) 上线性关系得到（与 forward_dynamics_min 同构，改为 Y*π）。
%
% 与 forwardDynamics(robot_limb, q, qd, tau) 原理一致性：
%   当 Y*π = inverseDynamics(robot_limb) 且 get_limb_theta_from_URDF(robot_limb)=π 时，两者求解的
%   是同一组 τ = M*qdd + h，故在无正则化、数值误差下应一致。一致性依赖 ReMatrix 与 Toolbox 的
%   运动学/重力/惯量约定一致。可用 set_limb_theta_to_URDF(robot_limb, π) 得到更新树后调用
%   forwardDynamics(robot_updated, q, qd, tau) 做交叉验证。
%
% 输入：
%   q, qd, tau - 1×n 或 n×1
%   pi_vec     - p×1 全参数（p=60 腿）
%   limb       - 'left_leg' | 'right_leg' | ...
%   para_order - 1 或 2
%
% 输出：
%   qdd - n×1 (rad/s^2)
%   diagOut - 诊断信息结构体（可选）

if nargin < 6
    para_order = 1;
end
if nargin < 7 || isempty(opts)
    opts = struct();
end

% 默认选项（尽量不改变物理，仅用于诊断与数值求解选择）
if ~isfield(opts, 'solver')
    % 'M' 或 'Msym'；Msym 仅用于解算输入，不在内部“偷偷修补”原 M
    opts.solver = 'Msym';
end
if ~isfield(opts, 'symmetry_tol_fro')
    opts.symmetry_tol_fro = 1e-8;
end
if ~isfield(opts, 'cond_warn')
    opts.cond_warn = 1e10;
end
if ~isfield(opts, 'check_qd_invariance')
    opts.check_qd_invariance = false;
end
if ~isfield(opts, 'regularize')
    % 若 true：在 Msym 上做最小特征值补丁（仅用于 debug/保护）
    opts.regularize = false;
end
if ~isfield(opts, 'regularize_min_eig')
    opts.regularize_min_eig = 1e-8;
end

q   = q(:).';
qd  = qd(:).';
tau = tau(:);
pi_vec = pi_vec(:);
n = numel(q);

ID_full = @(q_row, qd_row, qdd_row) ...
    ReMatrix_E1_limb_URDF(limb, q_row, qd_row, qdd_row, 1, para_order) * pi_vec;

qdd_zero = zeros(1, n);
qd_zero  = zeros(1, n);

% 基本维度检查（尽早暴露 pi_vec / q / tau 不一致）
Y0 = ReMatrix_E1_limb_URDF(limb, q, qd, qdd_zero, 1, para_order);
[nY, pY] = size(Y0);
assert(nY == n, 'forward_dynamics_full: regressor row mismatch (nY=%d, n=%d).', nY, n);
assert(numel(pi_vec) == pY, 'forward_dynamics_full: pi_vec length mismatch (numel(pi_vec)=%d, pY=%d).', numel(pi_vec), pY);
assert(numel(tau) == n, 'forward_dynamics_full: tau dimension mismatch (numel(tau)=%d, n=%d).', numel(tau), n);

tau_h = ID_full(q, qd, qdd_zero);      % h(q,qd)
tau_g = ID_full(q, qd_zero, qdd_zero); % g(q)

M = zeros(n, n);
for i = 1:n
    e_i = zeros(1, n);
    e_i(i) = 1;
    M(:, i) = ID_full(q, qd_zero, e_i) - tau_g;
end

% 诊断：对称性/正定性/条件数（不默认修补）
asym_fro = norm(M - M.', 'fro');
if asym_fro > opts.symmetry_tol_fro
    warning('forward_dynamics_full: mass matrix not symmetric, ||M-M''||_F = %.3e', asym_fro);
end

Msym = 0.5 * (M + M.');
eigMsym = eig(Msym);
min_eig = min(real(eigMsym));

% 非正定时：自动做 PSD 修补以保证 M\rhs 可解（π_rec/π_phys 等可能使惯量非物理）
if min_eig < opts.regularize_min_eig
    persistent warned_nonpd;
    if isempty(warned_nonpd), warned_nonpd = false; end
    if ~warned_nonpd
        warning('forward_dynamics_full: mass matrix not PD (min eig = %.3e), projecting to PSD (min_eig -> %.1e).', ...
            min_eig, opts.regularize_min_eig);
        warned_nonpd = true;
    end
    Msym = Msym + (opts.regularize_min_eig - min_eig) * eye(n);
    min_eig = opts.regularize_min_eig;
end

condMsym = cond(Msym);
if condMsym > opts.cond_warn
    warning('forward_dynamics_full: mass matrix ill-conditioned (on sym part), cond(Msym) = %.3e', condMsym);
end

qd_invar_fro = NaN;
if opts.check_qd_invariance
    % 检查 M 是否与 qd 无关（若不接近 0，优先怀疑 ReMatrix 实现而不是 FD 主线）
    M2 = zeros(n, n);
    tau0_qd = ID_full(q, qd, qdd_zero);
    for i = 1:n
        e_i = zeros(1, n); e_i(i) = 1;
        M2(:, i) = ID_full(q, qd, e_i) - tau0_qd;
    end
    qd_invar_fro = norm(M2 - M, 'fro');
    if qd_invar_fro > 1e-8
        warning('forward_dynamics_full: M(q) seems qd-dependent, ||M(qd)-M(0)||_F = %.3e', qd_invar_fro);
    end
end

rhs = tau - tau_h;

switch lower(string(opts.solver))
    case "m"
        qdd = M \ rhs;
    case "msym"
        qdd = Msym \ rhs;
    otherwise
        error('forward_dynamics_full: unknown opts.solver = %s (use ''M'' or ''Msym'').', string(opts.solver));
end
qdd = qdd(:);

if nargout > 1
    diagOut = struct();
    diagOut.M = M;
    diagOut.Msym = Msym;
    diagOut.tau_h = tau_h(:);
    diagOut.tau_g = tau_g(:);
    diagOut.rhs = rhs(:);
    diagOut.asym_fro = asym_fro;
    diagOut.eigMsym = eigMsym(:);
    diagOut.min_eig_Msym = min_eig;
    diagOut.condMsym = condMsym;
    diagOut.qd_invariance_fro = qd_invar_fro;
else
    diagOut = [];
end

end
