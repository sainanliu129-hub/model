%% run_full_dynamics_validation  整体流程：激励 → 采集 → 预处理 → 辨识 → β→π → ID/FD 对比
%
% 步骤概览（凡是“实机采集”的部分用注释说明，脚本只负责数学流程与绘图）：
%
%   Step 0: 生成激励轨迹（示意，实际在 Excitation_Trajectory_E1 工程中运行）
%   Step 1: 采集数据到 CSV（示意，控制器 + 记录系统完成）
%   Step 2: 数据预处理 + 最小参数辨识（run_min_param_id_from_csv）→ X_hat, index_base, avg_data, prep_opts
%   Step 3: β → 显式全参 π_rec（recover_full_params_from_beta），并与 URDF CAD 全参 π_cad 对比
%   Step 4: 逆动力学力矩对比（多模型，基于同一预处理/等效轨迹）：
%           τ_ref^{URDF}, Y*π_cad, Y_min*X_hat, Y*π_rec
%   Step 5: 正动力学加速度对比（多模型，同一预处理轨迹）：
%           qdd_meas(id 轨迹), FD_URDF, FD_min(β), FD_full(π_rec)
%   自检（Step 4 与 5 之间）：同一 (q,qd,τ) 下 FD(π_cad) 与 forwardDynamics(URDF) 应一致，
%           若不一致则问题在 forward_dynamics_full / ReMatrix / get_limb_theta_from_URDF。
%
% 用法：
%   1) 将采集到的 CSV 放到 app_root/data/excitation 下；
%   2) 在 MATLAB 中 cd 到 applications/Body_GravityPara_Iden/test；
%   3) 修改下方 csv_file 路径与 opts 预处理配置；
%   4) 运行 run_full_dynamics_validation。

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% Step 0. 生成激励轨迹（示意）
%{
exc_dir = fullfile(app_root, '..', 'Excitation_Trajectory_E1', 'excitation_trajectory');
addpath(exc_dir);
config = struct();
% ... 配置 move_axis / 关节上下限 / 周期 / 采样频率 / 谱阶 等 ...
[~, ~, refPos, refVel, refAcc, t_period] = generate_excitation_trajectory(config, ...
    'optimize', false, 'validate', false);
% refPos/refVel/refAcc 可用作离线仿真或周期对齐参考
%}

%% Step 1. 采集数据到 CSV（示意）
% 实机流程：
%   - 控制器执行 Step 0 的激励；
%   - 记录 time, pos_leg_l, vel_leg_l, torque_leg_l；
%   - 保存为 CSV，放到 data/excitation/ 目录。
%
% 本脚本假定已经有一条 CSV：
csv_file = fullfile(app_root, 'data', 'excitation', ...
    'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');

if ~isfile(csv_file)
    error('CSV 文件不存在: %s。请先将采集数据放到 data/excitation/ 目录下。', csv_file);
end

%% Step 2. 数据预处理 + 最小参数辨识
fprintf('\n===== Step 2: 数据预处理 + 最小参数辨识（run_min_param_id_from_csv） =====\n');

% 辨识选项（可按需要修改）：
opts = struct();
opts.mode = 'continuous';   % 'continuous'（默认）或 'cycle_average'
opts.use_preprocess_id = false;   % 先做预处理再连续窗辨识

% 连续窗选项（保证与 test_ReMatrix_E1_limb_URDF 等脚本一致更方便对比）
co = struct();
co.t_start_s        = 2.1;
co.t_end_s          = 4.1;
co.qd_lowpass_fc_Hz = 50;
co.qdd_smooth_half  = 15;
opts.continuous_opts = co;

% 正动力学/逆动力学验证时间窗（默认与辨识窗口一致，可单独修改）
fd_t_start_s = co.t_start_s;
fd_t_end_s   = co.t_end_s;
% 正动力学加速度对比仅用该时间窗内前 fd_n_max 个点（截取一小段以减耗时）；inf 表示用全部
fd_n_max = inf;

% 预处理选项：与 run_torque_comparison / run_min_param_id_from_csv 一致
prep = struct();
prep.t_start_s          = 2.1;
prep.t_end_s            = 4.1;
prep.sg_order           = 4;
prep.sg_frame           = 23;
prep.tau_lowpass_fc_Hz  = 15;
prep.tau_lowpass_order  = 2;
prep.do_plot            = false;
opts.prep_opts = prep;

% 若已经有结果文件，可以直接复用；否则跑一遍辨识
result_mat = fullfile(app_root, 'min_param_id_result.mat');
if ~isfile(result_mat)
    run_min_param_id_from_csv(csv_file, opts);
else
    fprintf('检测到已有 %s，直接复用（如需重跑，请手动删除该文件）。\n', result_mat);
    fprintf('  当前 csv_file = %s\n', csv_file);
    fprintf('  当前 opts.mode = %s, use_preprocess_id = %d\n', opts.mode, opts.use_preprocess_id);
end

ld = load(result_mat);
X_hat      = ld.X_hat;
index_base = ld.index_base;
avg_data   = ld.avg_data;
fprintf('  最小参数维度 p_min = %d\n', numel(X_hat));

%% Step 3. β → 全参 π_rec，并与 URDF CAD 全参 π_cad 对比
fprintf('\n===== Step 3: 最小参数 β → 全参 π_rec（recover_full_params_from_beta） =====\n');

limb = 'left_leg';
para_order = 1;
[robot_limb, n] = get_e1_limb_robot(limb);
pi_cad = get_limb_theta_from_URDF(robot_limb, para_order);   % URDF/CAD 全参

% 用 avg_data 轨迹构造 Y_full_id 与 Y_min_id（一次性批量计算，避免循环拼接）
q_bar_id   = avg_data.q_bar;
qd_bar_id  = avg_data.qd_bar;
qdd_bar_id = avg_data.qdd_bar;
Y_full_id  = ReMatrix_E1_limb_URDF(limb, q_bar_id, qd_bar_id, qdd_bar_id, 1, para_order);
Y_min_id = Y_full_id(:, index_base);

% 构造 K 并调用 recover_full_params_from_beta
K = pinv(Y_min_id) * Y_full_id;
beta_hat = X_hat(:);
[pi_rec, res_norm_rec] = recover_full_params_from_beta(K, beta_hat, pi_cad, 1e-2);

% π_cad → β_cad：用于验证“最小参数集”的 ID/FD 实现（同一组 CAD 参数下，Y_min*β_cad 应与 Y*π_cad 等价）
beta_cad = K * pi_cad(:);

fprintf('  recover_full_params_from_beta 残差范数 = %.3e\n', res_norm_rec);
fprintf('  π_cad 与 π_rec 的 max|差值| = %.3e\n', max(abs(pi_cad(:) - pi_rec(:))));
fprintf('  π_cad 与 π_rec 的 相对误差 ||Δπ||/||π_cad|| = %.3e\n', ...
    norm(pi_cad(:) - pi_rec(:)) / max(norm(pi_cad(:)), 1e-12));

%% Step 3b. 直接面向 FD 的 full 参数辨识：π_fd（42 维优化）
fprintf('\n===== Step 3b: 直接 FD 导向 full 参数辨识 (identify_full_params_for_fd) =====\n');

% 使用与辨识轨迹同一批 avg_data，抽样一部分点进行 full 参数优化
q_bar_fd   = avg_data.q_bar;
qd_bar_fd  = avg_data.qd_bar;
qdd_bar_fd = avg_data.qdd_bar;
tau_bar_fd = avg_data.tau_bar;

% 样本点与迭代大幅收紧，Step 3b 几分钟内结束（要更准可改大 n_qdd/max_iter）
M_fd_id = size(q_bar_fd, 1);
n_qdd   = min(15, M_fd_id);   % J_qdd 样本数，每步约 n_qdd 次 FD，越小越快
n_reg   = min(5, M_fd_id);   % J_M 样本数
idx_fd_sample = round(linspace(1, M_fd_id, n_qdd)).';
idx_reg_fd    = round(linspace(1, M_fd_id, n_reg)).';

opts_fd = struct();
opts_fd.w_tau = 1;
opts_fd.w_qdd = 10;
opts_fd.w_cad = 1;
opts_fd.w_M   = 10;
opts_fd.m_min_frac = 0.7;
opts_fd.m_max_frac = 1.3;
opts_fd.delta_c    = 0.02;
opts_fd.eps_I      = 1e-4;
opts_fd.eps_M      = 1e-6;
opts_fd.idx_qdd    = idx_fd_sample;
opts_fd.idx_reg    = idx_reg_fd;
opts_fd.max_iter   = 30;                     % 迭代少一点，先跑通
opts_fd.MaxFunctionEvaluations = 600;       % 到次数就停，避免拖很久
opts_fd.display   = 'iter';                  % 不刷屏，稍快一点

[pi_fd, info_fd] = identify_full_params_for_fd( ...
    q_bar_fd, qd_bar_fd, qdd_bar_fd, tau_bar_fd, ...
    pi_cad, limb, para_order, opts_fd);

fprintf('  [fd] fmincon exitflag = %d, fval = %.3e\n', info_fd.exitflag, info_fd.fval);
fprintf('  [fd] rmse_tau = %.4e, maxerr_tau = %.4e\n', info_fd.rmse_tau, info_fd.maxerr_tau);
fprintf('  [fd] rmse_qdd = %.4e, maxerr_qdd = %.4e\n', info_fd.rmse_qdd, info_fd.maxerr_qdd);
for i = 1:numel(info_fd.per_link)
    fprintf('  [fd] link %d: m=%.4f, c=[%.4f %.4f %.4f], minEigI=%.4e, condI=%.4e\n', ...
        i, info_fd.per_link(i).m, ...
        info_fd.per_link(i).c(1), info_fd.per_link(i).c(2), info_fd.per_link(i).c(3), ...
        info_fd.per_link(i).min_eig_I, info_fd.per_link(i).cond_I);
end
fprintf('  [fd] π_cad 与 π_fd 的 max|差值| = %.3e\n', max(abs(pi_cad(:) - pi_fd(:))));
fprintf('  [fd] π_cad 与 π_fd 的 相对误差 ||Δπ||/||π_cad|| = %.3e\n', ...
    norm(pi_cad(:) - pi_fd(:)) / max(norm(pi_cad(:)), 1e-12));

% 物理约束版 full 参数方案（加权 CAD 先验 + 质量/CoM/惯量约束，避免 π_phys 在 FD 上变差）
opts_phys = struct();
opts_phys.m_min_frac       = 0.5;
opts_phys.m_max_frac       = 1.5;
opts_phys.delta_c          = 0.02;
opts_phys.eps_pd           = 1e-4;
opts_phys.offdiag_max_frac = 0.5;
opts_phys.diag_dev_frac    = 0.8;
opts_phys.offdiag_dev_abs  = 0.05;
opts_phys.h_dev_abs        = 0.10;
opts_phys.w_m              = 20;
opts_phys.w_h              = 80;
opts_phys.w_Idiag          = 20;
opts_phys.w_Ioff           = 120;
opts_phys.algorithm        = 'sqp';
opts_phys.max_iter         = 500;
opts_phys.display          = 'iter';

[pi_phys, info_phys] = solve_full_params_physical(Y_full_id, Y_min_id, beta_hat, pi_cad, 10, opts_phys);

fprintf('  [phys] fmincon exitflag = %d\n', info_phys.exitflag);
fprintf('  [phys] fit_res_norm = %.4e, reg_res_norm = %.4e\n', info_phys.fit_res_norm, info_phys.reg_res_norm);
for i = 1:numel(info_phys.per_link)
    fprintf('  [phys] link %d: m=%.4f, c=[%.4f %.4f %.4f], minEigI=%.4e, condI=%.4e\n', ...
        i, info_phys.per_link(i).m, ...
        info_phys.per_link(i).c(1), info_phys.per_link(i).c(2), info_phys.per_link(i).c(3), ...
        info_phys.per_link(i).min_eig_I, info_phys.per_link(i).cond_I);
end
fprintf('  [phys] π_cad 与 π_phys 的 max|差值| = %.3e\n', max(abs(pi_cad(:) - pi_phys(:))));
fprintf('  [phys] π_cad 与 π_phys 的 相对误差 ||Δπ||/||π_cad|| = %.3e\n', ...
    norm(pi_cad(:) - pi_phys(:)) / max(norm(pi_cad(:)), 1e-12));

% 可选：导出参数辨识结果表（CAD / pi_rec / pi_phys），便于对比或制表
export_param_table = true;
if export_param_table
    n10 = numel(pi_cad);
    % 最小参数展开到 60 维（base 对应位置为 X_hat，其余填 NaN 表示无定义）
    x_hat_expand = nan(n10, 1);
    x_hat_expand(index_base) = X_hat(:);
    tbl = array2table([pi_cad(:), pi_phys(:), pi_rec(:), x_hat_expand], ...
        'VariableNames', {'CAD', 'pi_phys', 'pi_rec', 'x_hat_expand'});
    out_csv = fullfile(app_root, 'build', 'param_id_result_CAD_pi_phys_pi_rec.csv');
    if ~isfolder(fullfile(app_root, 'build')), mkdir(fullfile(app_root, 'build')); end
    writetable(tbl, out_csv);
    fprintf('  已导出参数表: %s\n', out_csv);
end

%% 按 link 分组的参数摘要表（重点看：谁的质量/CoM/惯量异常）
% 顺序每 link 10 维 [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]，据此算 CoM 与惯量特征值/条件数
n_links = numel(pi_cad) / 10;
summary_link = (1:n_links)';
m_cad = zeros(n_links,1); m_rec = zeros(n_links,1); m_phys = zeros(n_links,1);
cx_cad = zeros(n_links,1); cx_rec = zeros(n_links,1); cx_phys = zeros(n_links,1);
cy_cad = zeros(n_links,1); cy_rec = zeros(n_links,1); cy_phys = zeros(n_links,1);
cz_cad = zeros(n_links,1); cz_rec = zeros(n_links,1); cz_phys = zeros(n_links,1);
min_eig_I_cad = zeros(n_links,1); min_eig_I_rec = zeros(n_links,1); min_eig_I_phys = zeros(n_links,1);
cond_I_cad = zeros(n_links,1); cond_I_rec = zeros(n_links,1); cond_I_phys = zeros(n_links,1);
for i = 1:n_links
    idx = (i-1)*10 + (1:10);
    for tag = 1:3
        if tag == 1, p = pi_cad(idx); elseif tag == 2, p = pi_rec(idx); else, p = pi_phys(idx); end
        m = p(1); mx = p(2); my = p(3); mz = p(4);
        Ixx = p(5); Ixy = p(6); Ixz = p(7); Iyy = p(8); Iyz = p(9); Izz = p(10);
        if tag == 1
            m_cad(i) = m;
            if m > 1e-12, cx_cad(i)=mx/m; cy_cad(i)=my/m; cz_cad(i)=mz/m; end
            I3 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
            e = eig(I3); min_eig_I_cad(i) = min(real(e)); cond_I_cad(i) = cond(I3);
        elseif tag == 2
            m_rec(i) = m;
            if m > 1e-12, cx_rec(i)=mx/m; cy_rec(i)=my/m; cz_rec(i)=mz/m; end
            I3 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
            e = eig(I3); min_eig_I_rec(i) = min(real(e)); cond_I_rec(i) = cond(I3);
        else
            m_phys(i) = m;
            if m > 1e-12, cx_phys(i)=mx/m; cy_phys(i)=my/m; cz_phys(i)=mz/m; end
            I3 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
            e = eig(I3); min_eig_I_phys(i) = min(real(e)); cond_I_phys(i) = cond(I3);
        end
    end
end
tbl_link = table(summary_link, m_cad, m_rec, m_phys, ...
    cx_cad, cx_rec, cx_phys, cy_cad, cy_rec, cy_phys, cz_cad, cz_rec, cz_phys, ...
    min_eig_I_cad, min_eig_I_rec, min_eig_I_phys, cond_I_cad, cond_I_rec, cond_I_phys, ...
    'VariableNames', {'link','m_cad','m_rec','m_phys', ...
    'cx_cad','cx_rec','cx_phys','cy_cad','cy_rec','cy_phys','cz_cad','cz_rec','cz_phys', ...
    'min_eig_I_cad','min_eig_I_rec','min_eig_I_phys','cond_I_cad','cond_I_rec','cond_I_phys'});
fprintf('\n===== 按 link 参数摘要（m, CoM, min(eig(I)), cond(I)） =====\n');
disp(tbl_link);
out_summary = fullfile(app_root, 'build', 'param_id_per_link_summary.csv');
if ~isfolder(fullfile(app_root, 'build')), mkdir(fullfile(app_root, 'build')); end
writetable(tbl_link, out_summary);
fprintf('  已写入: %s\n', out_summary);

%% Step 4. 逆动力学力矩对比（实测/URDF/全参/最小参数/π_rec）
fprintf('\n===== Step 4: 逆动力学力矩对比（多模型） =====\n');

% 4.1 取经过预处理与连续窗后的“辨识轨迹”，并按 FD 验证时间窗再裁一遍
t_all   = avg_data.t_equiv(:);
q_all   = avg_data.q_bar;
qd_all  = avg_data.qd_bar;
qdd_all = avg_data.qdd_bar;
tau_all = avg_data.tau_bar;   % 若 use_preprocess_id=true，则已是 τ_id（扣过摩擦/转子）

mask_fd = (t_all >= fd_t_start_s) & (t_all <= fd_t_end_s);

t_id   = t_all(mask_fd);
q_id   = q_all(mask_fd, :);
qd_id  = qd_all(mask_fd, :);
qdd_id = qdd_all(mask_fd, :);
tau_id = tau_all(mask_fd, :);

M = size(q_id, 1);
% 正动力学对比仅用前 M_fd 个点（截取一小段以减耗时）
M_fd = min(M, fd_n_max);
if M_fd < M
    fprintf('  正动力学加速度对比仅用前 %d 个点（共 %d 点）以节省时间。\n', M_fd, M);
end
tau_ref_urdf = zeros(M, n);
tau_Y_cad    = zeros(M, n);
tau_Y_min    = zeros(M, n);
tau_Y_min_cad = zeros(M, n);   % 最小参数，但由 π_cad 映射得到：Y_min*β_cad，应与 Y*π_cad 等价
tau_Y_pirec  = zeros(M, n);
tau_Y_phys   = zeros(M, n);   % 物理约束版 full 参数

for k = 1:M
    qk   = q_id(k, :);
    qdk  = qd_id(k, :);
    qddk = qdd_id(k, :);

    % URDF 逆动力学：limb subtree
    tau_ref = inverseDynamics(robot_limb, qk, qdk, qddk).';
    tau_ref_urdf(k, :) = tau_ref;

    % 全参 CAD θ：Y*π_cad
    Y_one = ReMatrix_E1_limb_URDF(limb, qk, qdk, qddk, 1, para_order);
    tau_Y_cad(k, :) = (Y_one * pi_cad).';

    % 最小参数：Y_min*X_hat
    Y_min_k = Y_one(:, index_base);
    tau_Y_min(k, :) = (Y_min_k * X_hat).';

    % 最小参数（由 CAD 全参映射）：Y_min*β_cad（用于验证最小参数集的 ID 等价性）
    tau_Y_min_cad(k, :) = (Y_min_k * beta_cad).';

    % 恢复后的全参 π_rec：Y*π_rec
    tau_Y_pirec(k, :) = (Y_one * pi_rec).';
    % 物理约束版 π_phys：Y*π_phys
    tau_Y_phys(k, :)  = (Y_one * pi_phys).';
end

% 以 τ_ref^{URDF} 作为“参考模型力矩”做误差分析
err_tau_cad   = tau_Y_cad   - tau_id;
err_tau_min   = tau_Y_min   - tau_id;
err_tau_min_cad = tau_Y_min_cad - tau_id;
err_tau_pirec = tau_Y_pirec - tau_id;
err_tau_phys  = tau_Y_phys  - tau_id;

rmse_tau_cad   = sqrt(mean(err_tau_cad.^2, 1));
rmse_tau_min   = sqrt(mean(err_tau_min.^2, 1));
rmse_tau_min_cad = sqrt(mean(err_tau_min_cad.^2, 1));
rmse_tau_pirec = sqrt(mean(err_tau_pirec.^2, 1));
rmse_tau_phys  = sqrt(mean(err_tau_phys.^2, 1));
max_tau_cad    = max(abs(err_tau_cad),   [], 1);
max_tau_min    = max(abs(err_tau_min),   [], 1);
max_tau_min_cad = max(abs(err_tau_min_cad), [], 1);
max_tau_pirec  = max(abs(err_tau_pirec), [], 1);
max_tau_phys   = max(abs(err_tau_phys),  [], 1);

fprintf('  力矩 RMSE vs τ_{ref}^{URDF} (N·m):\n');
fprintf('    全参 CAD θ:      '); fprintf(' %.3f', rmse_tau_cad);   fprintf('\n');
fprintf('    最小参数 β:      '); fprintf(' %.3f', rmse_tau_min);   fprintf('\n');
fprintf('    最小参数 β_{cad}:' ); fprintf(' %.3f', rmse_tau_min_cad); fprintf('\n');
fprintf('    恢复全参 π_{rec}:'); fprintf(' %.3f', rmse_tau_pirec); fprintf('\n');
fprintf('    物理约束 π_{phys}:');fprintf(' %.3f', rmse_tau_phys);  fprintf('\n');

fprintf('  力矩 max|误差| vs τ_{ref}^{URDF} (N·m):\n');
fprintf('    全参 CAD θ:      '); fprintf(' %.3f', max_tau_cad);   fprintf('\n');
fprintf('    最小参数 β:      '); fprintf(' %.3f', max_tau_min);   fprintf('\n');
fprintf('    最小参数 β_{cad}:' ); fprintf(' %.3f', max_tau_min_cad); fprintf('\n');
fprintf('    恢复全参 π_{rec}:'); fprintf(' %.3f', max_tau_pirec); fprintf('\n');
fprintf('    物理约束 π_{phys}:');fprintf(' %.3f', max_tau_phys);  fprintf('\n');

% 绘图：6 关节力矩多模型对比
figure('Name', 'ID_力矩多模型对比');
plot_compare_6dof((1:M)', [tau_id, tau_ref_urdf, tau_Y_cad, tau_Y_min, tau_Y_min_cad, tau_Y_pirec, tau_Y_phys], 'torque', ...
    {'\tau_{meas/id}', '\tau_{ref}^{URDF}', 'Y\pi_{cad}', 'Y\_{min}X\_hat', 'Y\_{min}\beta_{cad}', 'Y\pi_{rec}', 'Y\pi_{phys}'});
sgtitle('逆动力学：实测/预处理力矩 vs URDF vs CAD全参 vs 最小参数(辨识) vs 最小参数(CAD映射) vs 恢复全参 vs 物理约束全参');

% 单点 ID 诊断（q=qd=qdd=0）：若 Y*π_cad 与 τ_urdf 明显不一致，应优先检查 ReMatrix / 参数提取 / 列顺序，
%  后续 FD 自检中出现的异常很大概率只是这一问题的放大。
q0 = zeros(1, n);
qd0 = zeros(1, n);
qdd0 = zeros(1, n);
tau_urdf_0  = inverseDynamics(robot_limb, q0, qd0, qdd0);
tau_urdf_0  = tau_urdf_0(:);
Y0 = ReMatrix_E1_limb_URDF(limb, q0, qd0, qdd0, 1, para_order);
tau_Y_cad_0 = Y0 * pi_cad;
tau_Y_cad_0 = tau_Y_cad_0(:);
err_id_0 = tau_Y_cad_0 - tau_urdf_0;
fprintf('\n  [单点 ID 诊断] (q,qd,qdd)=0: |Y*π_cad - τ_urdf| = ');
fprintf('%.3f ', abs(err_id_0));
fprintf('(N·m)\n');
if norm(err_id_0) > 0.1
    fprintf('  → 逆动力学在该基准点已明显不一致，建议优先跑 run_check_full_param_consistency_cad_only(trajectory_source=''random'') 与 run_diagnose_pi_Y_columns，\n');
    fprintf('    排查 ReMatrix / get_limb_theta_from_URDF（运动学 T0、重力 vd0、惯量列序等）。后续 FD 自检的异常很可能由此引起。\n');
end

%% 自检：同一组 CAD 参数下 FD(π_cad) 与 forwardDynamics(URDF) 必须一致
% 用 τ_ref_urdf = ID_urdf(q,qd,qdd_id) 保证同一 (q,qd,τ)，比较两种正动力学实现是否给出相同 qdd（仅前 M_fd 点）
fprintf('\n===== 自检：同一 (q,qd,\tau) 下 FD(\\pi_{cad}) vs forwardDynamics(URDF) =====\n');
qdd_fd_cad_check = zeros(M_fd, n);
qdd_fd_min_cad_check = zeros(M_fd, n);   % 最小参数 FD（由 π_cad 映射得到 β_cad）
qdd_urdf_check  = zeros(M_fd, n);
for k = 1:M_fd
    qk = q_id(k, :);
    qdk = qd_id(k, :);
    tauk = tau_ref_urdf(k, :);   % 与 URDF 自洽的 τ，便于隔离“两种 FD 实现是否一致”
    qdd_fd_cad_check(k, :) = forward_dynamics_full(qk, qdk, tauk, pi_cad, limb, para_order).';
    qdd_fd_min_cad_check(k, :) = forward_dynamics_min(qk.', qdk.', tauk(:), beta_cad, index_base, limb, para_order).';
    qdd_urdf_check(k, :)   = forwardDynamics(robot_limb, qk, qdk, tauk).';
end
err_fd_cad_vs_urdf = qdd_fd_cad_check - qdd_urdf_check;
err_fd_min_cad_vs_urdf = qdd_fd_min_cad_check - qdd_urdf_check;
rmse_fd_check = sqrt(mean(err_fd_cad_vs_urdf.^2, 1));
max_fd_check  = max(abs(err_fd_cad_vs_urdf), [], 1);
fprintf('  ||FD(\\pi_{cad}) - FD_{URDF}|| RMSE (rad/s^2): '); fprintf(' %.4e', rmse_fd_check); fprintf('\n');
fprintf('  max|FD(\\pi_{cad}) - FD_{URDF}| (rad/s^2):     '); fprintf(' %.4e', max_fd_check); fprintf('\n');
rel_fd = norm(err_fd_cad_vs_urdf, 'fro') / (norm(qdd_urdf_check, 'fro') + 1e-20);
fprintf('  相对误差 ||err||_F/||qdd_urdf||_F = %.4e\n', rel_fd);
if rel_fd < 1e-5
    fprintf('  通过：forward_dynamics_full(\\pi_{cad}) 与 forwardDynamics(URDF) 在该轨迹上一致。\n');
else
    fprintf('  未通过：同一组 CAD 下两种正动力学在该轨迹上存在可见差异。\n');
    fprintf('  常见根因包括：Step 4 中 Y*π_cad 与 τ_ref^{URDF} 已不一致（见全参 CAD θ 的 RMSE），或 ReMatrix/get_limb_theta_from_URDF 的约定不统一，\n');
    fprintf('  建议先跑 run_check_full_param_consistency_cad_only（trajectory_source=''random''）与 run_diagnose_pi_Y_columns，对 ReMatrix / get_limb_theta_from_URDF 的运动学 T0、重力 vd0、惯量列序等做针对性排查。\n');
end
rmse_fd_min_cad_check = sqrt(mean(err_fd_min_cad_vs_urdf.^2, 1));
max_fd_min_cad_check  = max(abs(err_fd_min_cad_vs_urdf), [], 1);
rel_fd_min_cad = norm(err_fd_min_cad_vs_urdf, 'fro') / (norm(qdd_urdf_check, 'fro') + 1e-20);
fprintf('  ||FD_{min}(\\beta_{cad}) - FD_{URDF}|| RMSE (rad/s^2): '); fprintf(' %.4e', rmse_fd_min_cad_check); fprintf('\n');
fprintf('  max|FD_{min}(\\beta_{cad}) - FD_{URDF}| (rad/s^2):     '); fprintf(' %.4e', max_fd_min_cad_check); fprintf('\n');
fprintf('  相对误差 ||err||_F/||qdd_urdf||_F = %.4e\n', rel_fd_min_cad);
if rel_fd_min_cad < 1e-5
    fprintf('  通过：forward_dynamics_min(\\beta_{cad}) 与 forwardDynamics(URDF) 一致。\n');
else
    fprintf('  未通过：同一组 CAD 下最小参数正动力学与 URDF 不一致（请检查 forward_dynamics_min 实现/约定）。\n');
end
figure('Name', '自检_FD(pi_cad)_vs_FD_URDF');
plot_compare_6dof((1:M_fd)', [qdd_urdf_check, qdd_fd_cad_check, qdd_fd_min_cad_check, err_fd_cad_vs_urdf, err_fd_min_cad_vs_urdf], 'qdd', ...
    {'qdd_{FD,URDF}', 'qdd_{FD(full,\\pi_{cad})}', 'qdd_{FD(min,\\beta_{cad})}', 'FD(full,\\pi_{cad})-FD_{URDF}', 'FD(min,\\beta_{cad})-FD_{URDF}'});
sgtitle('自检：同一 (q,qd,\tau) 下 FD(full, \pi_{cad}) / FD(min, \beta_{cad}) vs forwardDynamics(URDF)');

%% Step 5. 正动力学加速度对比（实测/CAD/最小参数/π_rec/π_phys/π_fd，仅前 M_fd 点）
% CAD 的 qdd 只算一次（forwardDynamics(URDF) 与 FD(π_cad)/FD(β_cad) 自检已通过，三者一致，后期统一用一个）
fprintf('\n===== Step 5: 正动力学加速度对比（多模型） =====\n');

fd_use_urdf_updated = false;

qdd_fd_cad  = zeros(M_fd, n);   % 仅一个 CAD 参考：forwardDynamics(robot_limb)
qdd_fd_min  = zeros(M_fd, n);
qdd_fd_full_rec = zeros(M_fd, n);
qdd_fd_phys = zeros(M_fd, n);
qdd_fd_fd   = zeros(M_fd, n);   % 直接 FD 导向辨识的 full 参数 π_fd

if fd_use_urdf_updated
    robot_rec  = set_limb_theta_to_URDF(robot_limb, pi_rec,  para_order, true);
    robot_phys = set_limb_theta_to_URDF(robot_limb, pi_phys, para_order, true);
    fprintf('  使用 set_limb_theta_to_URDF（PSD 投影）更新树，π_rec/π_phys 用 forwardDynamics 计算 FD。\n');
end

for k = 1:M_fd
    qk       = q_id(k, :);
    qdk      = qd_id(k, :);
    tauk_row = tau_id(k, :);
    tauk_col = tauk_row.';

    % CAD 参考 qdd（只算一次，与 FD(π_cad)/FD(β_cad) 等价，自检已验证）
    qdd_fd_cad(k, :) = forwardDynamics(robot_limb, qk, qdk, tauk_row).';

    % 最小参数 FD(β_hat)
    qdd_fd_min(k, :) = forward_dynamics_min(qk.', qdk.', tauk_col, ...
        X_hat, index_base, limb, para_order).';

    % 全参 FD(π_rec) / FD(π_phys)
    if fd_use_urdf_updated
        qdd_fd_full_rec(k, :) = forwardDynamics(robot_rec,  qk, qdk, tauk_row).';
        qdd_fd_phys(k, :)     = forwardDynamics(robot_phys, qk, qdk, tauk_row).';
    else
        qdd_fd_full_rec(k, :) = forward_dynamics_full(qk, qdk, tauk_row, pi_rec,  limb, para_order).';
        qdd_fd_phys(k, :)     = forward_dynamics_full(qk, qdk, tauk_row, pi_phys, limb, para_order).';
        qdd_fd_fd(k, :)       = forward_dynamics_full(qk, qdk, tauk_row, pi_fd,   limb, para_order).';
    end
end

err_qdd_cad  = qdd_fd_cad - qdd_id(1:M_fd, :);
err_qdd_min  = qdd_fd_min - qdd_id(1:M_fd, :);
err_qdd_full_rec = qdd_fd_full_rec - qdd_id(1:M_fd, :);
err_qdd_phys = qdd_fd_phys - qdd_id(1:M_fd, :);
err_qdd_fd   = qdd_fd_fd   - qdd_id(1:M_fd, :);

rmse_qdd_cad   = sqrt(mean(err_qdd_cad.^2, 1));
rmse_qdd_min   = sqrt(mean(err_qdd_min.^2, 1));
rmse_qdd_full_rec = sqrt(mean(err_qdd_full_rec.^2, 1));
rmse_qdd_phys = sqrt(mean(err_qdd_phys.^2, 1));
rmse_qdd_fd   = sqrt(mean(err_qdd_fd.^2,   1));
max_qdd_cad    = max(abs(err_qdd_cad), [], 1);
max_qdd_min    = max(abs(err_qdd_min), [], 1);
max_qdd_full_rec = max(abs(err_qdd_full_rec), [], 1);
max_qdd_phys  = max(abs(err_qdd_phys), [], 1);
max_qdd_fd    = max(abs(err_qdd_fd),   [], 1);

fprintf('  qdd RMSE (rad/s^2):\n');
fprintf('    FD(CAD):          '); fprintf(' %.3f', rmse_qdd_cad);   fprintf('\n');
fprintf('    FD(β):            '); fprintf(' %.3f', rmse_qdd_min);  fprintf('\n');
fprintf('    FD(π_{rec}):      '); fprintf(' %.3f', rmse_qdd_full_rec); fprintf('\n');
fprintf('    FD(π_{phys}):     '); fprintf(' %.3f', rmse_qdd_phys); fprintf('\n');
fprintf('    FD(π_{fd}):       '); fprintf(' %.3f', rmse_qdd_fd);   fprintf('\n');

fprintf('  qdd max|误差| (rad/s^2):\n');
fprintf('    FD(CAD):          '); fprintf(' %.3f', max_qdd_cad);   fprintf('\n');
fprintf('    FD(β):            '); fprintf(' %.3f', max_qdd_min);   fprintf('\n');
fprintf('    FD(π_{rec}):      '); fprintf(' %.3f', max_qdd_full_rec); fprintf('\n');
fprintf('    FD(π_{phys}):     '); fprintf(' %.3f', max_qdd_phys); fprintf('\n');
fprintf('    FD(π_{fd}):       '); fprintf(' %.3f', max_qdd_fd);   fprintf('\n');

figure('Name', 'FD_加速度多模型对比');
plot_compare_6dof((1:M_fd)', [qdd_id(1:M_fd,:), qdd_fd_cad, qdd_fd_min, qdd_fd_full_rec, qdd_fd_phys, qdd_fd_fd], 'qdd', ...
    {'qdd\_{id轨迹}', 'qdd\_{FD,CAD}', 'qdd\_{FD,\beta}', 'qdd\_{FD,\pi\_{rec}}', 'qdd\_{FD,\pi\_{phys}}', 'qdd\_{FD,\pi\_{fd}}'});
sgtitle('正动力学：实测/等效加速度 vs FD(CAD) vs FD(β) vs FD(π_{rec}) vs FD(π_{phys}) vs FD(π_{fd})');

figure('Name', 'FD_加速度多模型对比');
plot_compare_6dof((1:M_fd)', [qdd_id(1:M_fd,:), qdd_fd_cad, qdd_fd_phys, qdd_fd_fd], 'qdd', ...
    {'qdd\_{id轨迹}', 'qdd\_{FD,CAD}',  'qdd\_{FD,\pi\_{phys}}', 'qdd\_{FD,\pi\_{fd}}'});
sgtitle('正动力学：实测/等效加速度 vs FD(CAD) vs FD(π_{phys}) vs FD(π_{fd})');

fprintf('\n===== 全流程对比完成 =====\n');

