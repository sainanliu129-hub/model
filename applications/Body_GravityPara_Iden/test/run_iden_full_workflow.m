%% run_iden_full_workflow  一体化入口（薄封装）
% 目标：只保留“参数配置 + 调用主流程”，避免重复实现预处理/补偿/辨识逻辑。

clear; clc; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
if exist('ensure_body_gravity_para_iden_path', 'file')
    ensure_body_gravity_para_iden_path();
end

%% 1) 数据与时间窗
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
t_start_s = 2.1;
t_end_s   = 4.1;

%% 2) 预处理配置（低通 + 可选补偿）
prep_opts = struct();
prep_opts.t_start_s = t_start_s;
prep_opts.t_end_s = t_end_s;
prep_opts.q_lowpass_fc_Hz = 25;
prep_opts.q_lowpass_order = 2;
prep_opts.tau_lowpass_fc_Hz = 25;
prep_opts.tau_lowpass_order = 2;
prep_opts.do_plot = false;

% 可选补偿：在预处理阶段启用（解耦于辨识）
prep_opts.do_compensation = false;
prep_opts.load_friction_from_summary = false;  % true: 自动加载汇总表 friction + I_a
prep_opts.n_joints = 6;
prep_opts.row_for_joint = [0 0 0 0 0 0];

%% 3) 连续窗配置
continuous_opts = struct();
continuous_opts.t_start_s = t_start_s;
continuous_opts.t_end_s = t_end_s;
continuous_opts.qd_lowpass_fc_Hz = 0;
continuous_opts.qdd_smooth_half = 0;

%% 4) 调用主流程（辨识函数）
opts = struct();
opts.use_preprocess_id = true;
opts.prep_opts = prep_opts;
opts.continuous_opts = continuous_opts;

fprintf('===== 一体化流程（薄封装） =====\n');
fprintf('数据: %s\n', csv_file);
fprintf('时间窗: [%.2f, %.2f] s\n', t_start_s, t_end_s);
fprintf('补偿开关 do_compensation=%d, auto_load=%d\n', ...
    prep_opts.do_compensation, prep_opts.load_friction_from_summary);

run_min_param_id_from_csv(csv_file, opts);

fprintf('\n完成。后续可运行 run_full_dynamics_validation 或 run_debug_pi_phys_id_fd_plot。\n');

