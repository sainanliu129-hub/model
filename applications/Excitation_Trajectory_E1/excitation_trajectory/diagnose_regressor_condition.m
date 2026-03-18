%% diagnose_regressor_condition  打印条件数（与 baseGraIden 一致：rref 取独立列后 cond(W_min'*W_min)）
%
% 用法：
%   1) 无参：用一段简单轨迹（零位+小正弦）建 W，打印结论
%   2) diagnose_regressor_condition(refPos, refVel, refAcc) 用给定轨迹
%
% 算法：列归一化 → rref 取独立列 → cond_YY = cond(W_min'*W_min)
% 输出示例：
%   cond(W_min''*W_min) ≈ 1.2e+05 ，W 为 60 参，独立列数 42
%   σ_min = 2.1e-04 ，σ_max = 1.5e+01

function diagnose_regressor_condition(refPos, refVel, refAcc)
if nargin < 3 || isempty(refPos)
    % 缺省：左腿、短周期、少量点
    if isempty(which('get_e1_limb_robot'))
        repo = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
        addpath(fullfile(repo, 'applications', 'Body_GravityPara_Iden')); addpath(fullfile(repo, 'robot_model'));
    end
    period = 10;
    n = 200;
    t = linspace(0, period, n)';
    omega = 2*pi/period;
    mult = [1 1.3 0.9 1.1 0.85 1.15];
    refPos = 0.1 * [sin(mult(1)*omega*t), sin(mult(2)*omega*t), sin(mult(3)*omega*t), sin(mult(4)*omega*t), sin(mult(5)*omega*t), sin(mult(6)*omega*t)];
    refVel = zeros(size(refPos)); refAcc = zeros(size(refPos));
    for k = 1:6
        refVel(:,k) = 0.1 * (2*pi/period) * mult(k) * cos(mult(k)*omega*t);
        refAcc(:,k) = -0.1 * (2*pi/period)^2 * mult(k)^2 * sin(mult(k)*omega*t);
    end
end

output_type = 1;   % 60 参（BODY_JNT_CURRENT 辨识）
para_order  = 1;
limb        = 'left_leg';

Y = ReMatrix_E1_limb_URDF(limb, refPos, refVel, refAcc, output_type, para_order);
% 与 baseGraIden 一致：列归一化 → rref 取独立列 → cond(W_min'*W_min)
d = sqrt(sum(Y.^2, 1));
d(d < 1e-12) = 1;
W = Y ./ (ones(size(Y, 1), 1) * d);
[~, index_base] = rref(W);
if isempty(index_base)
    fprintf('rref 未得到独立列，请检查轨迹/矩阵。\n');
    return;
end
W_min = W(:, index_base);
G = W_min' * W_min;
cond_YY = cond(G);
fprintf('cond(W_min''*W_min) ≈ %.4e ，W 为 60 参，独立列数 %d\n', cond_YY, numel(index_base));
s = svd(W_min);
fprintf('σ_min = %.4e ，σ_max = %.4e\n', min(s), max(s));
end
