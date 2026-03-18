%% verify_error_source  验证全参误差 |τ_ref - Y*θ| 的可能来源
%
% 在零位及若干随机点上对比 τ_ref（inverseDynamics）、τ_Y（Y*θ），并做：
%   - 零位逐关节误差、重力力矩对照（gravityTorque）
%   - 重力符号/方向试探（翻转 Gravity 看 τ_ref 是否反号）
%   - Bodies 链序与关节轴打印（便于核对 base→tip、轴方向）
%
% 运行：在 test/ 下直接运行即可（脚本会 addpath principle 与 ensure_body_gravity_para_iden_path）。

clc;
app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if isempty(which('get_e1_limb_robot'))
    addpath(fullfile(app_root, '..', '..', 'robot_model'));
end
ensure_body_gravity_para_iden_path();

limb = 'left_leg';
para_order = 1;
[robot_limb, n] = get_e1_limb_robot(limb);
theta = get_limb_theta_from_URDF(robot_limb, para_order);

fprintf('===== 误差来源验证 =====\n');
fprintf('肢体: %s, n=%d, para_order=%d\n\n', limb, n, para_order);

%% 1) 零位：τ_ref、gravityTorque、Y*θ 三者对比
q0 = zeros(1, n);
qd0 = zeros(1, n);
qdd0 = zeros(1, n);

tau_ref = inverseDynamics(robot_limb, q0, qd0, qdd0);
tau_ref = tau_ref(:);

if exist('gravityTorque', 'file')
    tau_grav = gravityTorque(robot_limb, q0);
    tau_grav = tau_grav(:);
    fprintf('--- 1) 零位 (q=qd=qdd=0) ---\n');
    fprintf('  τ_ref (inverseDynamics):  %s\n', mat2str(tau_ref', 6));
    fprintf('  τ_grav (gravityTorque):  %s\n', mat2str(tau_grav', 6));
    fprintf('  |τ_ref - τ_grav|_∞ = %.6e（应≈0）\n', max(abs(tau_ref - tau_grav)));
else
    tau_grav = tau_ref;
    fprintf('--- 1) 零位 (q=qd=qdd=0) ---\n');
    fprintf('  τ_ref (inverseDynamics):  %s\n', mat2str(tau_ref', 6));
    fprintf('  （未调用 gravityTorque）\n');
end

Y0 = ReMatrix_E1_limb_URDF(limb, q0, qd0, qdd0, 1, para_order);
tau_Y = Y0 * theta;
tau_Y = tau_Y(:);
err0 = tau_ref - tau_Y;

fprintf('  τ_Y   (Y*θ):              %s\n', mat2str(tau_Y', 6));
fprintf('  误差 τ_ref - Y*θ 逐关节:  %s\n', mat2str(err0', 6));
fprintf('  |τ_ref - Y*θ|_∞ = %.6e N·m\n', max(abs(err0)));
fprintf('  Gravity (robot_limb):      [%.4g %.4g %.4g]\n', ...
    robot_limb.Gravity(1), robot_limb.Gravity(2), robot_limb.Gravity(3));
fprintf('  ReMatrix 内 vd0 = -Gravity（基座加速度）\n\n');

%% 2) 重力符号试探：翻转 Gravity 后 inverseDynamics 是否近似反号
g_orig = robot_limb.Gravity(:);
robot_limb.Gravity = -g_orig;   % 例如 [0 0 -9.81] -> [0 0 9.81]
tau_ref_flip = inverseDynamics(robot_limb, q0, qd0, qdd0);
tau_ref_flip = tau_ref_flip(:);
robot_limb.Gravity = g_orig;    % 恢复

fprintf('--- 2) 重力符号试探 ---\n');
fprintf('  τ_ref(Gravity):     %s\n', mat2str(tau_ref', 5));
fprintf('  τ_ref(-Gravity):   %s\n', mat2str(tau_ref_flip', 5));
fprintf('  τ_ref + τ_ref(-G): %s（应≈0，说明 Toolbox 重力项线性）\n', mat2str((tau_ref + tau_ref_flip)', 5));
fprintf('  Y*θ 更接近 τ_ref 还是 -τ_ref(-G)? 若接近后者则可能 vd0 符号与 Toolbox 反\n');
fprintf('  |τ_Y - τ_ref|_∞ = %.4e,  |τ_Y - (-τ_ref_flip)|_∞ = %.4e\n\n', ...
    max(abs(tau_Y - tau_ref)), max(abs(tau_Y - (-tau_ref_flip))));

%% 3) 链序与关节轴（base→tip、轴方向）
baseName = robot_limb.BaseName;
fprintf('--- 3) 链序与关节轴 ---\n');
fprintf('  BaseName: %s\n', baseName);
for i = 1:n
    bi = robot_limb.Bodies{i};
    ax = bi.Joint.JointAxis;
    if isempty(ax) || norm(ax) < 1e-10
        ax = [0; 0; 1];
    else
        ax = ax(:) / norm(ax);
    end
    fprintf('  Bodies{%d}: %s, Joint: %s, Axis: [%.4g %.4g %.4g]\n', ...
        i, bi.Name, bi.Joint.Name, ax(1), ax(2), ax(3));
end
fprintf('  （若链序非 base→tip 或某轴与 ReMatrix 假设相反，会导致系统性误差）\n\n');

%% 4) 若干随机点上的误差分布（是否以重力为主）
rng(42);
n_rand = 5;
q_rad = 0.5;
qd_rad = 0.3;
qdd_rad = 0.4;
err_inf = zeros(n_rand, 1);
tau_ref_n = zeros(n_rand, n);
tau_Y_n = zeros(n_rand, n);

fprintf('--- 4) 随机点误差 ---\n');
for k = 1:n_rand
    q = (rand(1, n) - 0.5) * 2 * q_rad;
    qd = (rand(1, n) - 0.5) * 2 * qd_rad;
    qdd = (rand(1, n) - 0.5) * 2 * qdd_rad;
    tau_ref_n(k, :) = inverseDynamics(robot_limb, q, qd, qdd);
    Yk = ReMatrix_E1_limb_URDF(limb, q, qd, qdd, 1, para_order);
    tau_Y_n(k, :) = (Yk * theta)';
    err_inf(k) = max(abs(tau_ref_n(k, :) - tau_Y_n(k, :)));
end
fprintf('  各点 |τ_ref - Y*θ|_∞: %s\n', mat2str(err_inf', 5));
fprintf('  零位误差: %.5e；随机点均值: %.5e\n', max(abs(err0)), mean(err_inf));
fprintf('  （若零位与随机点量级接近，偏置/重力相关；若随机点更大，动态项或链序也可能有问题）\n\n');

%% 5) 小结与建议
fprintf('--- 5) 小结与建议 ---\n');
fprintf('  - 若 |τ_Y - (-τ_ref_flip)| << |τ_Y - τ_ref|：怀疑 ReMatrix 中 vd0 符号（试改为 vd0 = +gravity）\n');
fprintf('  - 若零位误差大且逐关节分布不均：怀疑某连杆链序或关节轴方向\n');
fprintf('  - 若 gravityTorque 与 τ_ref 不一致：检查 Toolbox 与 DataFormat\n');
fprintf('===== 验证结束 =====\n');
