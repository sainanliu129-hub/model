function Result = identify_joint_friction(qd, i, K_tau, varargin)
% identify_joint_friction  拆下关节单独测试时辨识摩擦参数（静摩擦 τ_s、库伦 τ_c、粘滞 b）
%
% 前提：关节已从机器人上拆下，重力 g=0、耦合 C=0，故 τ_real = K_τ × i = τ_f(q̇)。
% 摩擦模型：τ_f = τ_c×sign(q̇) + b×q̇；静止时 |τ_f| ≤ τ_s。
% 支持不对称库伦：正向 τ_c_pos、负向 τ_c_neg（τ_f 正向 = τ_c_pos + b×q̇，负向 = τ_c_neg + b×q̇）。
%
% 辨识步骤：
%   1) 根据电机是否转动（|q̇|≈0）确定静摩擦 τ_s；
%   2) 仅用速度为正的数据拟合 τ_f = τ_c_pos + b×q̇；
%   3) 仅用速度为负的数据拟合 τ_f = τ_c_neg + b×q̇；
%   4) 正负两段 b 取平均；τ_c 为对称等效 (τ_c_pos - τ_c_neg)/2（兼容旧接口）。
%
% 输入:
%   qd     - 关节角速度，rad/s，列向量或行向量
%   i      - 电流（或直接为力矩），与 qd 同长度；τ_real = K_tau * i
%   K_tau  - 力矩系数，N·m/A；若 i 已是力矩则传 1
%   varargin - 可选名值对:
%     'EstimateTauS'   - true/false，是否根据静止段估计 τ_s（默认 true）
%     'EpsilonVel'     - 低速阈值 rad/s，|qd|<此值视为静止（默认 0.01）
%     'MinSpeed'       - 正/负速度拟合时 |qd| 需 >= 此值（默认 0.005）
%     'DoPlot'         - true/false，是否绘制拟合曲线（默认 false）
%     'JointName'      - 字符串，作图标题用（默认 ''）
%
% 输出:
%   Result - 结构体: .tau_c_pos, .tau_c_neg, .tau_c, .b, .tau_s, .tau_real, .tau_fit, .RMSE, .R2, .nUsed

narginchk(3, inf);
qd = qd(:);
i  = i(:);
if numel(qd) ~= numel(i)
    error('identify_joint_friction:LengthMismatch', 'qd 与 i 长度必须相同');
end

p = inputParser;
addParameter(p, 'EstimateTauS', true);
addParameter(p, 'EpsilonVel', 0.01);
addParameter(p, 'MinSpeed', 0.005);
addParameter(p, 'DoPlot', false);
addParameter(p, 'JointName', '');
parse(p, varargin{:});
opts = p.Results;

tau_real = K_tau * i;

% ---- 步骤 1：静摩擦 τ_s（电机不转时 |qd| < EpsilonVel）----
tau_s = NaN;
if opts.EstimateTauS
    idx_still = abs(qd) < opts.EpsilonVel;
    if any(idx_still)
        tau_s = mean(abs(tau_real(idx_still)));
    end
end

% ---- 步骤 2：速度为正 → 拟合 τ_f = τ_c + b×q̇ ----
idx_pos = qd >= opts.MinSpeed;
qd_pos = qd(idx_pos);
tau_pos = tau_real(idx_pos);
n_pos = numel(qd_pos);

% ---- 步骤 3：速度为负 → 拟合 τ_f = -τ_c + b×q̇ ----
idx_neg = qd <= -opts.MinSpeed;
qd_neg = qd(idx_neg);
tau_neg = tau_real(idx_neg);
n_neg = numel(qd_neg);

tau_c = NaN;
b     = NaN;
nUsed = 0;

if n_pos >= 2
    % τ = τ_c + b*qd  =>  [1, qd] * [τ_c; b]
    X_pos = [ones(n_pos, 1), qd_pos];
    theta_pos = X_pos \ tau_pos;
    tau_c_pos = theta_pos(1);
    b_pos     = theta_pos(2);
end
if n_neg >= 2
    % τ = τ_c_neg + b*qd（负向截距，可为负如 -1.4）
    X_neg = [ones(n_neg, 1), qd_neg];
    theta_neg = X_neg \ tau_neg;
    tau_c_neg = theta_neg(1);   % 负向截距
    b_neg     = theta_neg(2);
end

if n_pos >= 2 && n_neg >= 2
    b = (b_pos + b_neg) / 2;
    tau_c = (tau_c_pos - tau_c_neg) / 2;  % 对称等效，兼容旧接口
    nUsed = n_pos + n_neg;
elseif n_pos >= 2
    tau_c = tau_c_pos;
    tau_c_neg = -tau_c_pos;     % 仅正侧时假定对称
    b = b_pos;
    nUsed = n_pos;
elseif n_neg >= 2
    tau_c = -tau_c_neg;
    tau_c_pos = -tau_c_neg;     % 仅负侧时假定对称
    b = b_neg;
    nUsed = n_neg;
elseif n_pos == 1 && n_neg == 1
    % 退化拟合：仅 1 正 + 1 负，假定对称 τ_c_pos = -τ_c_neg，拟合 b 与 τ_c
    % τ_pos = τ_c + b*qd_pos, τ_neg = -τ_c + b*qd_neg => b = (τ_pos+τ_neg)/(qd_pos+qd_neg), τ_c = τ_pos - b*qd_pos
    qd_sum = qd_pos(1) + qd_neg(1);
    if abs(qd_sum) < 1e-9
        b = 0;
        tau_c_pos = tau_pos(1);
        tau_c_neg = -tau_c_pos;
    else
        b = (tau_pos(1) + tau_neg(1)) / qd_sum;
        tau_c_pos = tau_pos(1) - b * qd_pos(1);
        tau_c_neg = -tau_c_pos;
    end
    tau_c = tau_c_pos;  % 对称等效
    nUsed = 2;
    warning('identify_joint_friction:DegenerateFit', ...
        '正负速度各仅 1 点，为退化拟合；建议增加速度档位或减小 MinSpeed 以提高辨识精度');
else
    error('identify_joint_friction:InsufficientData', ...
        '正速度与负速度至少各需不少于 2 个点（|qd|>=MinSpeed），请增加数据或减小 MinSpeed');
end

% 拟合值（不对称）：正向 τ_c_pos + b*qd，负向 τ_c_neg + b*qd
tau_fit_all = (qd > 0) .* (tau_c_pos + b * qd) + (qd < 0) .* (tau_c_neg + b * qd) + (qd == 0) .* 0;
idx_fit = idx_pos | idx_neg;
resid_fit = tau_real(idx_fit) - tau_fit_all(idx_fit);
tau_fit_target = tau_real(idx_fit);
RMSE = sqrt(mean(resid_fit.^2));
ss_res = sum(resid_fit.^2);
ss_tot = sum((tau_fit_target - mean(tau_fit_target)).^2);
R2 = (ss_tot > 0) * (1 - ss_res / ss_tot) + (ss_tot == 0) * NaN;

% 输出结构体（tau_c_pos/neg 为正向/负向库伦截距，tau_c 为对称等效）
Result = struct();
Result.tau_c_pos = tau_c_pos;
Result.tau_c_neg = tau_c_neg;
Result.tau_c     = tau_c;
Result.b         = b;
Result.tau_s     = tau_s;
Result.tau_real  = tau_real;
Result.tau_fit   = tau_fit_all;
Result.RMSE      = RMSE;
Result.R2        = R2;
Result.nUsed     = nUsed;

% ---- 可选：绘图 ----
if opts.DoPlot
    plot_friction_fit(qd, tau_real, tau_fit_all, Result, opts.JointName);
end

end

function plot_friction_fit(qd, tau_real, tau_fit, Result, jointName)
% 绘制 (角速度, τ_real) 与拟合曲线（不对称库伦 τ_c_pos / τ_c_neg）
fig = figure('Name', 'Joint friction identification');
plot(qd, tau_real, 'b.', 'MarkerSize', 6);
hold on;
[qd_sorted, ord] = sort(qd);
tau_fit_sorted = (qd_sorted > 0) .* (Result.tau_c_pos + Result.b * qd_sorted) + ...
                 (qd_sorted < 0) .* (Result.tau_c_neg + Result.b * qd_sorted);
plot(qd_sorted, tau_fit_sorted, 'r-', 'LineWidth', 1.5);
xlabel('角速度 (rad/s)');
ylabel('力矩 (N\cdotm)');
legend('实测 \tau_{real}=K_\tau i', '拟合 \tau_f (\tau_{c+},\tau_{c-}+b\dot q)', 'Location', 'best');
grid on;
titleStr = sprintf('关节摩擦辨识: \\tau_{c+}=%.4f, \\tau_{c-}=%.4f N\\cdotm, b=%.4f, RMSE=%.4f, R^2=%.4f', ...
    Result.tau_c_pos, Result.tau_c_neg, Result.b, Result.RMSE, Result.R2);
if ~isempty(jointName)
    titleStr = [jointName ' | ' titleStr];
end
title(titleStr);
hold off;
end
