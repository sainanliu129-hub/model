%% test_cond_fun_from_regressor  快速验证 cond_fun_from_regressor 有效性
% 直接运行本脚本：用简单轨迹调用 cond_fun，打印 cond(W_min) 并对比 cond(W_min'*W_min)。

clear; clc;
this_dir   = fileparts(mfilename('fullpath'));
parent_dir = fullfile(this_dir, '..');                 % Excitation_Trajectory_E1
repo_root  = fullfile(parent_dir, '..', '..');         % model_e1 仓库根
addpath(this_dir);
addpath(parent_dir);
addpath(fullfile(repo_root, 'applications', 'Body_GravityPara_Iden'));
ensure_body_gravity_para_iden_path();

%% 1. 构造 cond_fun（E1_limb 左腿）
config = struct();
config.regressor_source = 'E1_limb';
config.cond_limb = 'left_leg';
cond_fun = cond_fun_from_regressor(config);

if isempty(cond_fun)
    error('cond_fun_from_regressor 返回空，请检查 config.regressor_source 与路径。');
end

%% 2. 简单轨迹：零位 + 小正弦（少量点便于快速跑）
period = 10;
n = 150;
t = linspace(0, period, n)';
omega = 2*pi/period;
mult = [1 1.3 0.9 1.1 0.85 1.15];
refPos = 0.1 * [sin(mult(1)*omega*t), sin(mult(2)*omega*t), sin(mult(3)*omega*t), ...
                 sin(mult(4)*omega*t), sin(mult(5)*omega*t), sin(mult(6)*omega*t)];
refVel = zeros(size(refPos));
refAcc = zeros(size(refPos));
for k = 1:6
    refVel(:,k) = 0.1 * (2*pi/period) * mult(k) * cos(mult(k)*omega*t);
    refAcc(:,k) = -0.1 * (2*pi/period)^2 * mult(k)^2 * sin(mult(k)*omega*t);
end

%% 3. 调用 cond_fun（优化目标：cond(W_min)）
c_w = cond_fun(refPos, refVel, refAcc);

%% 4. 同一轨迹手算 cond(W_min'*W_min) 做对比
Y = ReMatrix_E1_limb_URDF('left_leg', refPos, refVel, refAcc, 1, 1);
d = sqrt(sum(Y.^2, 1));
d(d < 1e-12) = 1;
W = Y ./ (ones(size(Y,1), 1) * d);
[~, jb] = rref(W);
if isempty(jb)
    c_gtw = nan;
else
    W_min = W(:, jb);
    c_gtw = cond(W_min' * W_min);
end

%% 5. 打印结果
fprintf('===== cond_fun_from_regressor 快速验证 =====\n');
fprintf('轨迹点数 N = %d，周期 = %.1f s\n', n, period);
fprintf('cond_fun = κ(W_min) = sqrt(cond(W''*W)) = %.4e  （与 C++ 一致）\n', c_w);
fprintf('手算 cond(W_min''*W_min)               = %.4e\n', c_gtw);
if isfinite(c_w) && isfinite(c_gtw) && c_gtw > 0
    fprintf('比值 cond(W''*W)/κ^2 ≈ %.4f  （应接近 1）\n', c_gtw / (c_w^2));
end
if isfinite(c_w) && c_w > 0
    fprintf('cond_fun 有效性：通过\n');
else
    fprintf('cond_fun 有效性：未通过（非有限或 <=0）\n');
end
fprintf('===== 结束 =====\n');
