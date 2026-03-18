function c = cond_value_E1_limb( limb, refPos, refVel, refAcc, output_type, para_order )
% cond_value_E1_limb  与 run_excitation_trajectory_standalone 所用条件数一致
%
% 与 cond_fun_from_regressor 中 local_cond_E1_limb 相同：列归一化 → rref 取独立列 →
% κ(W_min) = sqrt(cond(W_min'*W_min))，便于与 fmincon 方案对比效果。
%
% 输入: limb, refPos/refVel/refAcc 为 N×n (n=6 或 3)；output_type, para_order 同 ReMatrix_E1_limb_URDF。

if nargin < 5
    output_type = 1;
end
if nargin < 6
    para_order = 1;
end

MAX_POINTS_COND = 1500;

try
    [~, n_dof] = get_e1_limb_robot(limb);
    refPos = refPos(:, 1:min(n_dof, size(refPos,2)));
    refVel = refVel(:, 1:min(n_dof, size(refVel,2)));
    refAcc = refAcc(:, 1:min(n_dof, size(refAcc,2)));
    if size(refPos, 2) < n_dof
        refPos = [refPos, zeros(size(refPos,1), n_dof - size(refPos,2))];
        refVel = [refVel, zeros(size(refVel,1), n_dof - size(refVel,2))];
        refAcc = [refAcc, zeros(size(refAcc,1), n_dof - size(refAcc,2))];
    end
    N = size(refPos, 1);
    if N > MAX_POINTS_COND
        idx = round(linspace(1, N, MAX_POINTS_COND));
        refPos = refPos(idx, :);
        refVel = refVel(idx, :);
        refAcc = refAcc(idx, :);
    end
    Y = ReMatrix_E1_limb_URDF(limb, refPos, refVel, refAcc, output_type, para_order);
    c = regressor_condition_number(Y);
    if ~isfinite(c) || c <= 0
        c = 1e15;
    end
catch ME
    warning('cond_value_E1_limb: %s', ME.message);
    c = 1e15;
end
end
% 条件数计算已统一到 utility_function/regressor_condition_number.m
