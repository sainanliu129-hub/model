%% run_diagnose_pi_Y_columns  全参数与 Y 列对应关系排查
%
% 当自检 3 出现“质量 +10%% 后 Δτ=0”时，说明 π 与 Y 的对应关系或 Y 列结构有误。
% 本脚本：
%   1. 打印 π_cad 每 link 的 10 参（标注 m, hx, hy, hz, Ixx, ...）
%   2. 打印 Y_full 每列范数，并标注对应 (link, 参数名)
%   3. 单点有限差分：∂(Y*π)/∂π_j 是否等于 Y(:,j)
%   4. 静态重力测试：qd=0,qdd=0 下，仅改 link1 质量，τ 是否变化
%
% 约定（para_order=1）：每 link 10 列为 [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]；
%   Y 的列 (c-1)*10+(1:10) 对应 link c。
%
% 用法：cd 到 Body_GravityPara_Iden/test，运行 run_diagnose_pi_Y_columns

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

limb = 'left_leg';
para_order = 1;
param_names = {'m', 'mx', 'my', 'mz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz'};

%% 加载数据与构造 Y、π_cad（无 mat 时用随机轨迹，便于 A3 失败后立刻排查）
[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);
p = numel(pi_cad);
n_links = p / 10;

%% 0. 第一活动体与 body 顺序核对（确认“打印的 link1”是否即第一活动关节的 child link）
fprintf('\n===== 0. 第一活动体与 body 顺序（URDF/subtree 对照） =====\n');
fprintf('  limb = %s, subtree BaseName = %s\n', limb, robot_limb.BaseName);
fprintf('  get_limb_theta_from_URDF / ReMatrix 顺序 = robot_limb.Bodies{1}..Bodies{%d}\n', n);
fprintf('  下表：Bodies{i} = 第 i 个体，其 Joint 连接 Parent -> 本 body（child）\n');
for i = 1:n
    b = robot_limb.Bodies{i};
    pname = '';
    if ~isempty(b.Parent), pname = b.Parent.Name; end
    jname = ''; jtype = ''; jaxis = [NaN NaN NaN];
    if ~isempty(b.Joint), jname = b.Joint.Name; jtype = b.Joint.Type; jaxis = b.Joint.JointAxis(:)'; end
    com = b.CenterOfMass(:); if numel(com) < 3, com = [0;0;0]; end
    fprintf('  i=%d: body=%s  parent=%s  joint=%s (%s) axis=[%.4f %.4f %.4f]  m=%.4e  COM=(%.4e,%.4e,%.4e)\n', ...
        i, b.Name, pname, jname, jtype, jaxis(1), jaxis(2), jaxis(3), b.Mass, com(1), com(2), com(3));
end
% 第一活动关节：subtree 下第一个非 fixed 的 joint，其 child 即“第一活动体”
first_actuated_child = '';
for i = 1:n
    b = robot_limb.Bodies{i};
    if ~isempty(b.Joint) && ~strcmpi(b.Joint.Type, 'fixed')
        first_actuated_child = b.Name;
        fprintf('  第一活动关节: %s (parent=%s -> child=%s)，child 即“第一活动体”。\n', ...
            b.Joint.Name, b.Parent.Name, b.Name);
        break;
    end
end
fprintf('  结论: 打印的 link1 (Bodies{1}=%s) 与 第一活动体(child)=%s 一致则顺序正确。\n', ...
    robot_limb.Bodies{1}.Name, first_actuated_child);

result_mat = fullfile(app_root, 'min_param_id_result.mat');
if isfile(result_mat)
    ld = load(result_mat);
    q_bar   = ld.avg_data.q_bar;
    qd_bar  = ld.avg_data.qd_bar;
    qdd_bar = ld.avg_data.qdd_bar;
    index_base = ld.index_base;
    [M, n] = size(q_bar);
    [~, Y_full, ~, ~] = build_K_from_regressor(limb, q_bar, qd_bar, qdd_bar, index_base, para_order);
    fprintf('===== 全参数与 Y 列对应关系排查（轨迹来自 mat） =====\n');
else
    % 无 mat：随机轨迹 + identify_min 得 index_base，再构造 Y_full
    fprintf('未找到 %s，使用随机轨迹构造 Y_full（便于 CAD 自检失败后立刻排查）。\n', result_mat);
    rng(44);
    n_random = 80;
    q_bar   = (rand(n_random, n) - 0.5) * 1.2;
    qd_bar  = (rand(n_random, n) - 0.5) * 1.0;
    qdd_bar = (rand(n_random, n) - 0.5) * 1.0;
    tau_bar = zeros(n_random, n);
    for k = 1:n_random
        tau_bar(k,:) = inverseDynamics(robot_limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:))';
    end
    M = n_random;
    avg_data.q_bar = q_bar; avg_data.qd_bar = qd_bar; avg_data.qdd_bar = qdd_bar; avg_data.tau_bar = tau_bar; avg_data.tau_std = zeros(M, n);
    [~, index_base, ~] = identify_min(avg_data, limb, struct('para_order', para_order, 'use_wls', false));
    [~, Y_full, ~, ~] = build_K_from_regressor(limb, q_bar, qd_bar, qdd_bar, index_base, para_order);
    fprintf('===== 全参数与 Y 列对应关系排查（轨迹: 随机 %d 点） =====\n', M);
end
fprintf('  n_links = %d, 每 link 10 参, para_order = %d\n', n_links, para_order);

%% 1. 打印 π_cad 按 link 的 10 参
fprintf('\n--- 1. π_cad 按 link（约定: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz） ---\n');
for i = 1:n_links
    idx = (i-1)*10 + (1:10);
    blk = pi_cad(idx);
    fprintf('  link%d: ', i);
    for j = 1:10
        fprintf(' %s=%.4e', param_names{j}, blk(j));
    end
    fprintf('\n');
end

%% 2. Y_full 每列范数及对应 (link, 参数)
fprintf('\n--- 2. Y_full 各列范数 ||Y(:,j)||（列 j 对应 link/参数） ---\n');
col_norms = sqrt(sum(Y_full.^2, 1));
fprintf('  j    link param    ||Y(:,j)||\n');
for j = 1:p
    link_j = floor((j-1)/10) + 1;
    param_j = mod(j-1, 10) + 1;
    fprintf(' %3d  %2d   %-4s   %.4e\n', j, link_j, param_names{param_j}, col_norms(j));
end
fprintf('  列范数为 0 或极小时可能原因：1) 参数映射错误；2) 该参数对该关节结构上不可观；3) 当前轨迹未激励到。\n');

%% 3. 单点有限差分：∂(Y*π)/∂π_j 应等于 Y(:,j)（取第一个时刻的 6 行）
fprintf('\n--- 3. 单点有限差分验证（第 1 个时刻，关节 1） ---\n');
k = 1;
rows_k = (k-1)*n + (1:n);
Y_k = Y_full(rows_k, :);   % n×p
eps_fd = 1e-6 * max(1, max(abs(pi_cad)));
max_err_fd = 0;
worst_j = 1;
for j = 1:min(10, p)   % 先只检查 link1 的 10 列
    e_j = zeros(p, 1);
    e_j(j) = 1;
    tau_plus  = Y_k * (pi_cad + eps_fd * e_j);
    tau_minus = Y_k * (pi_cad - eps_fd * e_j);
    fd_col = (tau_plus - tau_minus) / (2 * eps_fd);
    err_fd = norm(fd_col - Y_k(:, j));
    if err_fd > max_err_fd
        max_err_fd = err_fd;
        worst_j = j;
    end
    if j <= 4
        fprintf('  j=%d (%s): ||(τ(+εej)-τ(-εej))/(2ε) - Y(:,j)|| = %.4e\n', j, param_names{j}, err_fd);
    end
end
fprintf('  link1 前 10 列中最大差分误差 = %.4e (列 j=%d)\n', max_err_fd, worst_j);
if max_err_fd < 1e-8
    fprintf('  有限差分与 Y 列一致，Y*π 实现正确。\n');
else
    fprintf('  存在偏差，需检查 Y 组装或 π 顺序。\n');
end

%% 4. 静态重力测试：qd=0, qdd=0，分别改 link1/link2 的 m、mx，看 ||Δτ||
fprintf('\n--- 4. 静态重力测试（qd=0, qdd=0）---\n');
q_static = q_bar(1, :);
qd_zero  = zeros(1, n);
qdd_zero = zeros(1, n);
Y_static = ReMatrix_E1_limb_URDF(limb, q_static, qd_zero, qdd_zero, 1, para_order);  % 6×60

tau0 = Y_static * pi_cad;
delta_rel = 0.1;   % 相对扰动 10%
% 列索引: link1 m=1, link1 mx=2; link2 m=11, link2 mx=12
lab = {'link1 m ', 'link1 mx', 'link2 m ', 'link2 mx'};
j_list = [1, 2, 11, 12];
fprintf('  扰动 10%%（若该参≈0 则加 1e-6），看 ||Δτ|| 是否 >0：\n');
fprintf('  参数       π(j)         delta      ||τ_pert - τ_0||   ||Y(:,j)||\n');
norm_link1 = 0; norm_link2 = 0;
for idx = 1:4
    j = j_list(idx);
    val = pi_cad(j);
    delta = val * delta_rel;
    if abs(val) < 1e-12, delta = 1e-6; end
    pi_pert = pi_cad;
    pi_pert(j) = pi_cad(j) + delta;
    tau_pert = Y_static * pi_pert;
    dtau = tau_pert - tau0;
    ntau = norm(dtau);
    if j <= 10, norm_link1 = max(norm_link1, ntau); else, norm_link2 = max(norm_link2, ntau); end
    fprintf('  %-10s %.4e   %.4e   %.4e   %.4e\n', lab{idx}, val, delta, ntau, norm(Y_static(:, j)));
end
fprintf('  若改 link1 后 ||Δτ||≈0、改 link2 后 ||Δτ||>0：可能列序/第一活动体错位。\n');
fprintf('  若 link1/link2 静态扰动均 ≈0：对第 1 关节为竖直 yaw、重力沿 z 时属正常（绕 z 无重力矩）。\n');
if norm_link1 < 1e-12 && norm_link2 > 1e-8
    fprintf('  当前: link1 扰动 τ 不变、link2 扰动 τ 变 -> 建议核对 body 顺序或 Trel{1}。\n');
elseif norm_link1 > 1e-8
    fprintf('  当前: link1 扰动 τ 变化 -> link1 列参与正常。\n');
else
    fprintf('  当前: link1/link2 静态扰动均 ≈0 -> 若 joint1 为竖轴、重力沿 z，属结构正常。\n');
end

%% 5. 小结：列范数近似为 0 的列（区分：映射错误 / 结构不可观 / 未激励）
zero_cols = find(col_norms < 1e-12);
fprintf('\n--- 5. 列范数近似为 0 的列 ---\n');
if isempty(zero_cols)
    fprintf('  无。\n');
else
    fprintf('  列索引: '); fprintf(' %d', zero_cols); fprintf('\n');
    for j = zero_cols(:)'
        link_j = floor((j-1)/10) + 1;
        fprintf('    j=%d -> link%d %s\n', j, link_j, param_names{mod(j-1,10)+1});
    end
end
fprintf('  说明：0 可能来自 1) 参数列序错位 2) 该参数对该关节结构上不可观 3) 轨迹未激励。\n');
fprintf('  若仅 link1 前 9 列为 0、link1 Izz 非 0：当第 1 关节为竖轴且重力沿 z 时，属第 2 类（结构不可观），非必然错误。\n');

%% 6. 检查 A：joint1-link1 块是否恒为 [0..0 nonzero]（多取几点）
fprintf('\n--- 6. 检查 A：joint1 对 link1 的块 Y(1,1:10) 在多时刻是否仅第 10 列非零 ---\n');
n_check = min(5, size(Y_full, 1) / n);
for ki = 1:n_check
    row1 = (ki - 1) * n + 1;
    blk = Y_full(row1, 1:10);
    nz = sum(abs(blk) > 1e-12);
    fprintf('  时刻 %d: 非零元个数 = %d（若恒为 1 且仅列 10，则 link1 对 joint1 为结构仅 Izz）\n', ki, nz);
end

%% 7. 检查 B：joint2~6 对 link1 的列是否按结构为 0
fprintf('\n--- 7. 检查 B：joint2~6 行在 link1 列块上的范数 ---\n');
joint1_rows = (0:(M-1)) * n + 1;
rows_j2to6 = setdiff(1:(M*n), joint1_rows);
norm_j2to6_link1 = norm(Y_full(rows_j2to6, 1:10), 'fro');
fprintf('  ||Y(关节2~6 行, link1 列)||_F = %.4e（标准串联下应为 0，因 link1 只贡献 joint1）\n', norm_j2to6_link1);

fprintf('\n排查建议：若确认属结构不可观则无需改列序；若有错位再查 get_limb_theta_from_URDF / ReMatrix 与 Trel{1}、JointAxis。\n');
