function [pi_rec, res_norm] = recover_full_params_from_beta(K, beta_hat, pi_cad, lambda)
% recover_full_params_from_beta  由最小参数 β 与 CAD 先验恢复全参数 π（baseline 无物理约束）
%
% 优化：min_π |K*π - β_hat|^2 + λ|π - π_cad|^2
% 闭式解：π_rec = (K'*K + λ*I)^{-1} * (K'*beta_hat + λ*pi_cad)
%
% 输入：
%   K        - r×p，β = K*π（由 build_K_from_regressor 得到）
%   beta_hat - r×1 辨识得到的最小参数（即 X_hat）
%   pi_cad   - p×1 CAD/URDF 全参数（与 ReMatrix 列序一致，如 get_limb_theta_from_URDF）
%   lambda   - 正则系数，越大越贴近 π_cad（默认 1e-2）
%
% 输出：
%   pi_rec   - p×1 恢复的全参数
%   res_norm - 可选，|K*pi_rec - beta_hat|（最小参数残差范数）

if nargin < 4 || isempty(lambda)
    lambda = 1e-2;
end

beta_hat = beta_hat(:);
pi_cad   = pi_cad(:);
p = size(K, 2);
r = size(K, 1);
if numel(beta_hat) ~= r
    error('recover_full_params_from_beta: beta_hat 长度 %d 应与 K 行数 %d 一致。', numel(beta_hat), r);
end
if numel(pi_cad) ~= p
    error('recover_full_params_from_beta: pi_cad 长度 %d 应与 K 列数 %d 一致。', numel(pi_cad), p);
end

pi_rec = (K'*K + lambda*eye(p)) \ (K'*beta_hat + lambda*pi_cad);
if nargout >= 2
    res_norm = norm(K*pi_rec - beta_hat);
end

end
