function c = regressor_condition_number(W)
% regressor_condition_number  回归矩阵条件数：列归一化 → rref 取独立列 → κ(W_min)
%
% 与 Body_GravityPara_Iden 中 cond_fun_from_regressor、cond_value_E1_limb 一致：
% κ(W_min) = sqrt(cond(W_min'*W_min))，用于激励轨迹优化等。
%
% 输入：W 为 (M*n)×p 的回归矩阵（如 Y = ReMatrix_E1_limb_URDF 输出）
% 输出：c 为条件数，若无可逆列则返回 inf

W = normalize_columns(W);
[~, index_base] = rref(W);
if isempty(index_base)
    c = inf;
    return;
end
W_min = W(:, index_base);
G = W_min' * W_min;
c = sqrt(cond(G));
if ~isfinite(c) || c <= 0
    c = inf;
end
end

function W = normalize_columns(W)
d = sqrt(sum(W.^2, 1));
d(d < 1e-12) = 1;
W = W ./ (ones(size(W, 1), 1) * d);
end
