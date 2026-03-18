function [X_hat, index_base, metrics] = identify_min(avg_data, limb, opts)
% identify_min  基于相位平均后的数据做最小参数辨识
%
% 用法：
%   [X_hat, index_base, metrics] = identify_min(avg_data, 'left_leg');
%   [X_hat, index_base, metrics] = identify_min(avg_data, 'left_leg', struct('use_wls', false));
%
% 输入：
%   avg_data - 由 cycle_average 得到的结构体，至少包含：
%              .q_bar (M×n), .qd_bar (M×n), .qdd_bar (M×n), .tau_bar (M×n), .tau_std (M×n 或 M×1)
%   limb     - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   opts     - 可选，第三参可省略。结构体字段：
%              .para_order     (默认 1)
%              .use_wls        (logical, 默认 true)
%              .lambda         (正则化系数，默认 0 = 无正则)
%              .X0             (先验参数向量，与 X_hat 维度一致；lambda>0 时建议提供)
%              .friction_params (可选) 摩擦补偿：结构体 .tau_c_pos, .tau_c_neg, .b（各 1×n 或标量）
%              .I_a            (可选) 电机转子惯量 (kg·m²)，1×n 或标量
%              若提供，辨识前用 τ_rb = τ_meas - τ_friction - τ_rotor 做最小二乘。
%
% 输出：
%   X_hat     - 最小参数估计 (p_min×1)
%   index_base- 线性无关列索引（Y_min = Y(:, index_base)）
%   metrics   - 结构体，包含：
%               .rmse_per_joint, .maxerr_per_joint, .tau_pred, .tau_meas

if nargin < 3
    opts = struct();
end
if ~isfield(opts, 'para_order'), opts.para_order = 1; end
if ~isfield(opts, 'use_wls'),    opts.use_wls = true; end
if ~isfield(opts, 'lambda'),     opts.lambda = 0; end
if ~isfield(opts, 'X0'),         opts.X0 = []; end

q_bar   = avg_data.q_bar;
qd_bar  = avg_data.qd_bar;
qdd_bar = avg_data.qdd_bar;
tau_bar = avg_data.tau_bar;

[M, n] = size(q_bar);
if any(size(qd_bar) ~= [M,n]) || any(size(qdd_bar) ~= [M,n]) || any(size(tau_bar) ~= [M,n])
    error('identify_min: avg_data 中 q_bar/qd_bar/qdd_bar/tau_bar 尺寸不一致。');
end

% 0. 可选：辨识前减去摩擦与电机转子惯量力矩，用 τ_rb = τ_meas - τ_friction - τ_rotor 拟合
tau_meas_original = tau_bar;
fp = []; if isfield(opts, 'friction_params') && ~isempty(opts.friction_params), fp = opts.friction_params; end
ia = []; if isfield(opts, 'I_a') && ~isempty(opts.I_a), ia = opts.I_a; end
if ~isempty(fp) || ~isempty(ia)
    tau_comp = subtract_friction_rotor_torque(qd_bar, qdd_bar, fp, ia);
    tau_bar = tau_bar - tau_comp;
end

% 1. 计算全参回归矩阵 Y（(M·n)×60），与 test_ReMatrix 完全一致：逐点调用 ReMatrix 再堆叠
Y_full = [];
for k = 1:M
    q_k   = q_bar(k, :);
    qd_k  = qd_bar(k, :);
    qdd_k = qdd_bar(k, :);
    Y_one = ReMatrix_E1_limb_URDF(limb, q_k, qd_k, qdd_k, 1, opts.para_order);
    Y_full = [Y_full; Y_one];
end

% 2. 列归一化 + rank + QR 列主元得到 index_base（与 test_ReMatrix_E1_limb_URDF 一致）
%    test_ReMatrix 中 H = pinv(Y_min)*Y 为全参→最小参数映射，β = H*θ 满足 Y_min*β ≈ Y*θ（有 θ 时用）。
%    辨识时无 θ，用 Y_min\τ_meas 求 X_hat，与验收脚本中 X_hat_meas = Y_min\tau_meas_stack 一致。
col_norm = sqrt(sum(Y_full.^2, 1));
col_norm(col_norm < 1e-12) = 1;
W = Y_full ./ (ones(size(Y_full,1),1) * col_norm);

r = rank(W);                      % SVD 定秩
[~, ~, piv] = qr(W, 'vector');    % QR 列主元，W(:,piv)=Q*R
index_base = sort(piv(1:r));
if isempty(index_base)
    error('identify_min: 未得到独立列，请检查激励轨迹与回归矩阵。');
end

Y_min = Y_full(:, index_base);
p_min = numel(index_base);

% 3. 组装 A, b（行顺序必须与 ReMatrix_E1_limb_URDF 一致：按点堆叠 [点1(6); 点2(6); ...]）
%    ReMatrix 输出 Y 行为 point-major；tau_bar(:) 为 column-major（关节优先），故用 reshape(tau_bar',[],1)
A = Y_min;                          % (M·n)×p_min
b = reshape(tau_bar', [], 1);       % (M·n)×1，按点堆叠，与 Y 行序一致

% 4. 权重（WLS，权重顺序与 b 一致：按点堆叠）
if isfield(avg_data, 'tau_std') && ~isempty(avg_data.tau_std) && opts.use_wls
    tau_std = avg_data.tau_std;
    if isvector(tau_std)
        tau_std = repmat(tau_std(:), 1, n);
    end
    if any(size(tau_std) ~= [M,n])
        error('identify_min: tau_std 尺寸应为 M×n 或 M×1。');
    end
    sigma_vec = reshape(tau_std', [], 1);   % 与 b 同序：按点堆叠
    eps_sigma = 1e-6;
    w = 1 ./ (sigma_vec + eps_sigma);       % (M·n)×1
else
    w = ones(M*n, 1);
end

% 将权重吸收进 A, b：A_w = diag(w)*A, b_w = diag(w)*b
Aw = bsxfun(@times, w, A);
bw = w .* b;

% 5. 正则化（可选）
if opts.lambda > 0
    lambda = opts.lambda;
    if isempty(opts.X0)
        X0 = zeros(p_min,1);
    else
        X0 = opts.X0;
        if numel(X0) ~= p_min
            error('identify_min: X0 维度应为 %d。', p_min);
        end
        X0 = X0(:);
    end
    % 等价于解 [Aw; sqrt(lambda)I] * X = [bw; sqrt(lambda)X0]
    Areg = [Aw; sqrt(lambda)*eye(p_min)];
    breg = [bw; sqrt(lambda)*X0];
    X_hat = Areg \ breg;
else
    X_hat = Aw \ bw;
end

% 6. 计算预测力矩与误差指标（tau_pred_vec 为按点堆叠 [点1(6); 点2(6); ...]，需 reshape 成 M×n 且 tau_pred(k,j)=点k关节j）
tau_pred_vec = A * X_hat;      % (M·n)×1，point-major
tau_pred = reshape(tau_pred_vec, [n, M])';   % 先 [n,M] 则列 k = 点 k，再转置得 (k,j)=点k关节j
tau_meas = tau_meas_original;   % 始终为原始实测力矩（辨识用 b 可能已为 τ_rb）

err = tau_pred - tau_meas;
rmse_per_joint   = sqrt(mean(err.^2, 1));
maxerr_per_joint = max(abs(err), [], 1);

metrics = struct();
metrics.rmse_per_joint   = rmse_per_joint;
metrics.maxerr_per_joint = maxerr_per_joint;
metrics.tau_pred         = tau_pred;
metrics.tau_meas         = tau_meas;
if exist('tau_comp', 'var') && ~isempty(tau_comp)
    metrics.tau_rb = tau_bar;   % 补偿后的刚体力矩（τ_meas - τ_friction - τ_rotor）
end

end

