function qdd = forward_dynamics_min(q, qd, tau, X_hat, index_base, limb, para_order)
% forward_dynamics_min  基于最小参数集的正动力学（不依赖完整 URDF 惯性）
%
% 用法：
%   qdd = forward_dynamics_min(q, qd, tau, X_hat, index_base, 'left_leg', 1);
%
% 输入：
%   q          - 关节位置 (n×1 或 1×n)
%   qd         - 关节速度 (n×1 或 1×n)
%   tau        - 关节力矩 (n×1 或 1×n)
%   X_hat      - 最小参数估计 (p_min×1)
%   index_base - 对应 ReMatrix 列的最小参数索引
%   limb       - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   para_order - 与 ReMatrix_E1_limb_URDF 一致（1 或 2）
%
% 输出：
%   qdd        - 关节加速度 (n×1)

if nargin < 7
    para_order = 1;
end

q   = q(:).';
qd  = qd(:).';
tau = tau(:);

n = numel(q);
if numel(qd) ~= n || numel(tau) ~= n
    error('forward_dynamics_min: q, qd, tau 维度不一致。');
end

X_hat = X_hat(:);

% 嵌套函数：最小参数逆动力学 ID_min(q, qd, qdd)
    function tau_id = ID_min(q_row, qd_row, qdd_row)
        % q_row, qd_row, qdd_row 为 1×n 行向量
        Y_one = ReMatrix_E1_limb_URDF(limb, q_row, qd_row, qdd_row, 1, para_order);  % n×(10n)
        Y_min = Y_one(:, index_base);                                                % n×p_min
        tau_id = Y_min * X_hat;                                                      % n×1
    end

% 1. 偏置项 h(q, qd) = ID_min(q, qd, 0)
qdd_zero = zeros(1, n);
tau_h = ID_min(q, qd, qdd_zero);    % n×1

% 2. 重力项 g(q) = ID_min(q, 0, 0)
qd_zero = zeros(1, n);
tau_g = ID_min(q, qd_zero, qdd_zero);  % n×1

% 3. 构造质量矩阵列：M(:,i) = ID_min(q, 0, e_i) - g(q)
M = zeros(n, n);
for i = 1:n
    e_i = zeros(1, n);
    e_i(i) = 1;
    tau_col = ID_min(q, qd_zero, e_i);   % n×1
    M(:, i) = tau_col - tau_g;
end

% 数值稳健性：对称化 + 轻微正定性修正（如必要）
M = 0.5 * (M + M.');

% 简单正定性修正：若最小特征值过小或负，则抬升
[V, D] = eig((M+M.')/2);
eigvals = diag(D);
min_eig = min(real(eigvals));
if min_eig < 1e-8
    delta = 1e-8 - min_eig;
    M = M + delta * eye(n);
end

% 4. 正动力学：qdd = M \ (tau - h)
rhs = tau - tau_h;
qdd = M \ rhs;

% 输出列向量
qdd = qdd(:);

end

