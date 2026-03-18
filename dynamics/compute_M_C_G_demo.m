% compute_M_C_G_demo  演示：用 get_e1_full_robot_limb_indices 得到索引后对比单链与整机 M、C、G
%
% 调用 get_e1_full_robot_limb_indices() 得到各肢体在整机中的实际索引，
% 再在相同位形下对比左腿单链的 M、C、G 与整机对应块。
%
% 用法：在工程根目录或已 addpath robot_algorithm/Dynamics/E1 与 RobotModel 后运行
%   compute_M_C_G_demo

% 确保能调用 get_e1_full_robot、get_e1_limb_robot、get_e1_full_robot_limb_indices、compute_e1_*
clear all
base = fileparts(mfilename('fullpath'));
root = fullfile(base, '..', '..');
addpath(genpath(fullfile(root, 'robot_algorithm')));
addpath(genpath(fullfile(root, 'utility_function')));

fprintf('========== E1 单链 vs 整机 M、C、G 对比 demo ==========\n\n');

%% 1. 得到各肢体在整机中的实际索引
[idx_ll, idx_rl, idx_la, idx_ra] = get_e1_full_robot_limb_indices();
fprintf('整机肢体索引（由质量矩阵块匹配得到）:\n');
fprintf('  左腿: %s\n', mat2str(idx_ll));
fprintf('  右腿: %s\n', mat2str(idx_rl));
fprintf('  左臂: %s\n', mat2str(idx_la));
fprintf('  右臂: %s\n\n', mat2str(idx_ra));

%% 2. 整机 18 维 M、C、G（零位 + 一随机位形）
q_full  = zeros(18, 1);
qd_full = zeros(18, 1);
[M_full, C_full, G_full] = compute_e1_dynamics(q_full, qd_full);

q_rand  = (rand(18, 1) - 0.5) * 0.3;
qd_rand = (rand(18, 1) - 0.5) * 0.1;
[M_full_rand, C_full_rand, G_full_rand] = compute_e1_dynamics(q_rand, qd_rand);

%% 3. 左腿单链 M、C、G（相同位形：取整机对应分量）
q_left   = q_full(idx_ll);
qd_left  = qd_full(idx_ll);
[M_ll, C_ll, G_ll] = compute_e1_limb_dynamics('left_leg', q_left, qd_left);

fprintf('--- 零位 q=0, qd=0 ---\n');
fprintf('左腿单链:  M_ll 6×6, G_ll 6×1\n');
fprintf('整机块:    M_full(idx_ll,idx_ll) 与 G_full(idx_ll)\n');
err_M_ll = norm(M_full(idx_ll, idx_ll) - M_ll, 'fro');
err_G_ll = norm(G_full(idx_ll) - G_ll);
fprintf('   || M_full(idx_ll,idx_ll) - M_ll ||_F = %.2e\n', err_M_ll);
fprintf('   || G_full(idx_ll) - G_ll ||       = %.2e\n', err_G_ll);
fprintf('   (C 在零速下为 0，不单独比较)\n\n');

%% 4. 随机位形下左腿单链 vs 整机块
q_left_r   = q_rand(idx_ll);
qd_left_r  = qd_rand(idx_ll);
[M_ll_r, C_ll_r, G_ll_r] = compute_e1_limb_dynamics('left_leg', q_left_r, qd_left_r);

fprintf('--- 随机位形 ---\n');
fprintf('左腿单链 vs 整机对应块:\n');
fprintf('   || M_full(idx_ll,idx_ll) - M_ll ||_F = %.2e\n', norm(M_full_rand(idx_ll, idx_ll) - M_ll_r, 'fro'));
fprintf('   || C_full(idx_ll) - C_ll ||       = %.2e\n', norm(C_full_rand(idx_ll) - C_ll_r));
fprintf('   || G_full(idx_ll) - G_ll ||       = %.2e\n', norm(G_full_rand(idx_ll) - G_ll_r));

fprintf('\n说明：若误差很小，说明单链与该块一致；若存在肢体间耦合，非对角块非零，单链与整机块会有差异。\n');
