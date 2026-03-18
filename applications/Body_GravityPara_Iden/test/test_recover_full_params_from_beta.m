%% test_recover_full_params_from_beta  校核 β→π_full 恢复链路（纯 URDF）
%
% 用 θ_urdf 生成 τ_ref 和 β_urdf，再通过当前恢复算法得到 π_rec，
% 对比 Y_min*β_urdf 与 Y_full*π_rec 是否一致。
%
% 在 Body_GravityPara_Iden/test 下运行：
%   test_recover_full_params_from_beta

clc; clear; close all;
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

limb = 'left_leg';
para_order = 1;

[robot_limb, n] = get_e1_limb_robot(limb);
theta_urdf = get_limb_theta_from_URDF(robot_limb, para_order);

fprintf('===== β→π_full 恢复链路纯 URDF 校核 =====\n');
fprintf('肢体: %s, n=%d, para_order=%d\n\n', limb, n, para_order);

%% 1) 构造一批纯 URDF 轨迹点 (q,qd,qdd)
rng(123);
nTest = 50;
q_rad  = 0.6;
qd_rad = 0.5;
qdd_rad = 0.8;
q_all   = (rand(nTest, n) - 0.5) * 2 * q_rad;
qd_all  = (rand(nTest, n) - 0.5) * 2 * qd_rad;
qdd_all = (rand(nTest, n) - 0.5) * 2 * qdd_rad;

YY_full = [];
tau_ref_stack = zeros(nTest * n, 1);

for k = 1:nTest
    q   = q_all(k,:);
    qd  = qd_all(k,:);
    qdd = qdd_all(k,:);

    % 参考 torque: 直接用 URDF θ_urdf 与 ReMatrix 计算 Y_full*θ_urdf
    Y_one = ReMatrix_E1_limb_URDF(limb, q, qd, qdd, 1, para_order);
    tau_ref = Y_one * theta_urdf;

    YY_full = [YY_full; Y_one];
    tau_ref_stack((k-1)*n + (1:n)) = tau_ref(:);
end

fprintf('构造轨迹点数: %d，总回归矩阵大小: %dx%d\n', nTest, size(YY_full,1), size(YY_full,2));

%% 2) 从 Y_full 构造 Y_min、β_urdf（与 test_ReMatrix_E1_limb_URDF 相同）
col_norm = sqrt(sum(YY_full.^2, 1));
col_norm(col_norm < 1e-12) = 1;
W = YY_full ./ (ones(size(YY_full,1), 1) * col_norm);

r = rank(W);
[~, ~, piv] = qr(W, 'vector');
index_base = sort(piv(1:r));

Y_min = YY_full(:, index_base);
p_min = numel(index_base);

fprintf('最小参数维度 p_min = %d / %d\n', p_min, 10*n);

% β_urdf 由 τ_ref = Y_min*β_urdf 拟合得到
beta_urdf = Y_min \ tau_ref_stack;

tau_min_from_beta = Y_min * beta_urdf;
err_beta_vs_ref = norm(tau_min_from_beta - tau_ref_stack, inf);
fprintf('URDF: max |Y_min*β_urdf - τ_ref|_∞ = %.6e N·m\n', err_beta_vs_ref);

%% 3) 通过 K-正则约束从 β_urdf 得到 π_rec_full（废弃 S-based 示例）
%
% 使用 recover_full_params_from_beta 中的约束：
%   min_π ||K*π - β_hat||^2 + λ||π - π_cad||^2
% 其中 K 满足 β = K*π, Y_full = Y_min*K。

K = pinv(Y_min) * YY_full;          % 与 build_K_from_regressor 相同构造
lambda = 1e-2;
[pi_rec, res_norm] = recover_full_params_from_beta(K, beta_urdf, theta_urdf, lambda);
fprintf('recover_full_params_from_beta: ||K*pi_rec - beta_urdf|| = %.6e\n', res_norm);

%% 4) 比较 Y_full*pi_rec 与 Y_min*β_urdf 以及 Y_full*θ_urdf

tau_full_rec  = YY_full * pi_rec;
tau_full_urdf = YY_full * theta_urdf;

err_rec_vs_min  = norm(tau_full_rec  - tau_min_from_beta, inf);
err_rec_vs_urdf = norm(tau_full_rec  - tau_full_urdf,    inf);
err_min_vs_urdf = norm(tau_min_from_beta - tau_full_urdf, inf);

fprintf('\n=== Torque 等价性检查 ===\n');
fprintf('max |Y_full*pi_rec - Y_min*β_urdf|_∞ = %.6e N·m\n', err_rec_vs_min);
fprintf('max |Y_full*pi_rec - Y_full*θ_urdf|_∞ = %.6e N·m\n', err_rec_vs_urdf);
fprintf('max |Y_min*β_urdf - Y_full*θ_urdf|_∞ = %.6e N·m\n', err_min_vs_urdf);

fprintf('\n校核完成。\n');

%% 5) 可视化：逐关节 torque 对比与误差

x_pts = (1:nTest).';
tau_ref_mat      = reshape(tau_ref_stack, n, nTest).';        % nTest×n
tau_min_mat      = reshape(tau_min_from_beta, n, nTest).';
tau_full_rec_mat = reshape(tau_full_rec,      n, nTest).';

figure('Name','recover_full_params_torque_compare');
plot_compare_6dof(x_pts, [tau_ref_mat, tau_min_mat, tau_full_rec_mat], 'torque', ...
    {'\tau_{ref} = Y_{full}\theta_{urdf}', '\tau_{min} = Y_{min}\beta_{urdf}', '\tau_{rec} = Y_{full}\pi_{rec}'});
sgtitle('β→π_{full} 恢复校核：三种 torque 对比');

figure('Name','recover_full_params_torque_error');
err_min_vs_ref  = tau_min_mat      - tau_ref_mat;
err_rec_vs_ref  = tau_full_rec_mat - tau_ref_mat;
plot_compare_6dof(x_pts, [err_min_vs_ref, err_rec_vs_ref], 'torque', ...
    {'Y_{min}\beta_{urdf} - Y_{full}\theta_{urdf}', 'Y_{full}\pi_{rec} - Y_{full}\theta_{urdf}'});
sgtitle('β→π_{full} 恢复校核：相对 URDF 的 torque 误差');


