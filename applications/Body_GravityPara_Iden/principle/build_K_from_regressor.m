function [K, Y_full, Y_min, errK] = build_K_from_regressor(limb, q_bar, qd_bar, qdd_bar, index_base, para_order)
% build_K_from_regressor  由堆叠回归矩阵构造 β = K*π 的矩阵 K
%
% 关系：τ = Y_full*π = Y_min*β，且 β = K*π，故 Y_full = Y_min*K，即 K = pinv(Y_min)*Y_full。
%
% 输入：
%   limb       - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   q_bar      - M×n 关节位置 (rad)
%   qd_bar     - M×n 关节速度 (rad/s)
%   qdd_bar    - M×n 关节加速度 (rad/s^2)
%   index_base - 最小参数列索引（与 identify_min 输出一致）
%   para_order - 1 或 2，与 ReMatrix_E1_limb_URDF 一致（默认 1）
%
% 输出：
%   K      - r×p，β = K*π，r = length(index_base)，p = 10*n
%   Y_full - (M*n)×p 堆叠全参回归矩阵
%   Y_min  - (M*n)×r = Y_full(:, index_base)
%   errK   - 重构相对误差 norm(Y_full - Y_min*K,'fro')/norm(Y_full,'fro')

if nargin < 6 || isempty(para_order)
    para_order = 1;
end

[M, n] = size(q_bar);
if size(qd_bar,1) ~= M || size(qdd_bar,1) ~= M
    error('build_K_from_regressor: q_bar/qd_bar/qdd_bar 行数须一致。');
end

p = 10 * n;
Y_full = [];
for k = 1:M
    Y_one = ReMatrix_E1_limb_URDF(limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, para_order);
    Y_full = [Y_full; Y_one];
end

Y_min = Y_full(:, index_base);
K = pinv(Y_min) * Y_full;
errK = norm(Y_full - Y_min*K, 'fro') / (norm(Y_full, 'fro') + 1e-20);

end
