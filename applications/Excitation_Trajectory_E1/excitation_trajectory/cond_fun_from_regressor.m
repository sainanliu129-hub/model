function cond_fun = cond_fun_from_regressor(config)
% cond_fun_from_regressor  由辨识矩阵（ReMatrix）构造条件数函数，供激励轨迹优化使用
%
% 根据 config 选择 ReMatrix 计算观测矩阵 W（即 Y）。先列归一化 W̃，rref 取独立列得 W_min。
% 优化目标返回 cond(W_min)，不返回 cond(W_min'*W_min)，避免 WᵀW 将条件数平方放大。
%
% 优化与打印均为 cond(W_min)，即 rref 得到的独立列（Y_min）的条件数。
% 条件数仍可能很大（1e6~1e8）：rref 只保证列线性无关，不保证“数值上分离好”；
% 这些独立列仍可近似共线 → σ_min 很小 → cond=σ_max/σ_min 放大。非实现错误。
% 若 max_acceleration 已足够大，主因多为 60 全参结构冗余（独立列仍近乎相关），可运行 diagnose_regressor_condition 看 σ_min。
%
% 输入 config 字段：
%   .regressor_source - 'RobotDynPara' | 'E1_limb'
%   RobotDynPara 时：.Robot（含 .DH, .gravity），依赖 calMatrix_kinHomTrans
%   E1_limb 时：    .cond_limb = 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   可选：.regressor_output_type（默认 1），.regressor_para_order（默认 1）
%
% 输出：
%   cond_fun - 函数句柄 @(refPos, refVel, refAcc) cond_number
%   refPos/refVel/refAcc 为 N×6 (rad, rad/s, rad/s^2)

if ~isfield(config, 'regressor_source')
    cond_fun = [];
    return;
end

src = config.regressor_source;
ot = 1;
if isfield(config, 'regressor_output_type') && ~isempty(config.regressor_output_type)
    ot = config.regressor_output_type;
end
po = 1;
if isfield(config, 'regressor_para_order') && ~isempty(config.regressor_para_order)
    po = config.regressor_para_order;
end

if strcmpi(src, 'RobotDynPara')
    if ~isfield(config, 'Robot') || isempty(config.Robot)
        warning('cond_fun_from_regressor: regressor_source=RobotDynPara 但未提供 config.Robot，返回空');
        cond_fun = [];
        return;
    end
    Robot = config.Robot;
    cond_fun = @(refPos, refVel, refAcc) local_cond_RobotDynPara(Robot, refPos, refVel, refAcc, ot, po);
elseif strcmpi(src, 'E1_limb')
    limb = 'left_leg';
    if isfield(config, 'cond_limb') && ~isempty(config.cond_limb)
        limb = config.cond_limb;
    end
    cond_fun = @(refPos, refVel, refAcc) local_cond_E1_limb(limb, refPos, refVel, refAcc, ot, po);
else
    warning('cond_fun_from_regressor: 未知 regressor_source ''%s''', src);
    cond_fun = [];
end
end

function c = local_cond_RobotDynPara(Robot, refPos, refVel, refAcc, ot, po)
try
    refPos = refPos(:, 1:min(6, size(refPos,2)));
    refVel = refVel(:, 1:min(6, size(refVel,2)));
    refAcc = refAcc(:, 1:min(6, size(refAcc,2)));
    if size(refPos, 2) < 6
        refPos = [refPos, zeros(size(refPos,1), 6 - size(refPos,2))];
        refVel = [refVel, zeros(size(refVel,1), 6 - size(refVel,2))];
        refAcc = [refAcc, zeros(size(refAcc,1), 6 - size(refAcc,2))];
    end
    Y = ReMatrix_RobotDynPara(Robot, refPos, refVel, refAcc, ot, po);
    c = regressor_condition_number(Y);
catch
    c = inf;
end
end

function c = local_cond_E1_limb(limb, refPos, refVel, refAcc, ot, po)
MAX_POINTS_COND = 1500;
try
    [~, n_dof] = get_e1_limb_robot(limb);   % 腿 6，臂 3
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
    Y = ReMatrix_E1_limb_URDF(limb, refPos, refVel, refAcc, ot, po);
    c = regressor_condition_number(Y);
    if ~isfinite(c) || c <= 0, c = 1e15; end
catch ME
    warning('cond_fun_from_regressor(E1_limb): %s', ME.message);
    c = 1e15;
end
end

% 条件数计算已统一到 utility_function/regressor_condition_number.m
