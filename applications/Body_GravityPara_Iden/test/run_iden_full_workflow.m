%% run_iden_full_workflow  一体化运行：预处理 + 摩擦/惯量补偿 + 最小参数辨识
%
% 包含《辨识前数据预处理流程》中的完整步骤：
%   Step 1  时间轴统一与裁剪有效段
%   Step 2-3 从位置 q 经 Savitzky-Golay 得到 q_s, qd_s, qdd_s（同源）
%   Step 4  对 τ 零相位低通（Butterworth + filtfilt）
%   Step 5  τ_id = τ_s − J_eq·qdd_s − τ_f(qd_s) − τ_bias
%   Step 6  用 (q_s, qd_s, qdd_s, τ_id) 做最小参数辨识
%
% 使用步骤：
%   1. 修改下方「一、配置」中的 csv_file、时间窗、开关等
%   2. 在 MATLAB 中 cd 到 Body_GravityPara_Iden，运行本脚本（F5）

clear;
clc;
close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

%% ========== 一、配置 ==========
% 数据文件（可改为 data/跑步/ 下 CSV）
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195346_exctra_sim.csv');
% csv_file = fullfile(app_root, 'data', '跑步', 'PD-M1-v0_multi_joint_20260226-182548.csv');

% 有效段时间 (s)，只在该段内做预处理与辨识
t_start_s = 2.1;
t_end_s   = 4.1;

% 是否做「辨识前预处理」（推荐 true：从 q 得 q/qd/qdd、τ 低通、构造 τ_id）
use_preprocess_id = false;

% 是否从「全部电机参数汇总.xlsx」读取 stribeck_viscous + I_a（需先完成 Friction_Iden，表在 Friction_Iden/data 或 build 下）
load_friction_from_summary = true;
n_joints = 6;
% 关节→表行映射（仅当 load_friction_from_summary=true 时有效）：
%   row_for_joint(j)=k 表示关节 j 用汇总表第 k 行（stribeck 筛选后的行号，从 1 起）；
%   0 表示该关节摩擦/惯量全为 0。[]=默认按表顺序 1→1,2→2,…，表行不足的关节填 0。
% row_for_joint = [0 1 1 10 6 6];   % 例：[1 2 3 4 5 0] 表示关节1~5用表第1~5行，关节6用0
row_for_joint = [0 0 0 0 0 0];   % 例：[1 2 3 4 5 0] 表示关节1~5用表第1~5行，关节6用0

% 预处理子选项（仅当 use_preprocess_id = true 时生效）
prep_opts = struct();
prep_opts.t_start_s         = t_start_s;
prep_opts.t_end_s           = t_end_s;
prep_opts.sg_order          = 4;       % Savitzky-Golay 多项式阶次
prep_opts.sg_frame          = 41;      % SG 窗长（奇数），500Hz 约 0.08s
prep_opts.tau_lowpass_fc_Hz = 15;      % 力矩低通截止 (Hz)
prep_opts.tau_lowpass_order = 2;       % Butterworth 阶数
prep_opts.do_plot           = true;    % 每步后画图对比（裁剪/SG/滤波/τ_id）
prep_opts.plot_joints       = [1 2 3]; % 参与绘图的关节（避免图过多）
% 可选：首尾裁掉点数，或异常幅值阈值
% prep_opts.trim_start_n = 50;
% prep_opts.trim_end_n   = 50;
% prep_opts.max_abs_q    = 3.5;
% prep_opts.max_abs_tau  = 50;

%% ========== 二、组装 opts 并调用主流程 ==========
continuous_opts = struct('t_start_s', t_start_s, 't_end_s', t_end_s, ...
    'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);

opts = struct();
opts.mode              = 'continuous';
opts.continuous_opts   = continuous_opts;
opts.use_preprocess_id = use_preprocess_id;
opts.prep_opts         = prep_opts;
opts.load_friction_from_summary = load_friction_from_summary;
opts.n_joints          = n_joints;
opts.row_for_joint    = row_for_joint;

if load_friction_from_summary
    friction_iden_dir = fullfile(app_root, '..', 'Friction_Iden');
    if isfolder(friction_iden_dir) && isempty(which('load_friction_stribeck_from_summary'))
        addpath(friction_iden_dir);
    end
    load_opts = struct();
    if ~isempty(opts.row_for_joint), load_opts.row_for_joint = opts.row_for_joint; end
    [opts.friction_params, opts.I_a] = load_friction_stribeck_from_summary(n_joints, load_opts);
    fprintf('已从「全部电机参数汇总」表加载 stribeck_viscous + I_a（n_joints=%d）\n', n_joints);
    % 预处理里会用摩擦与惯量构造 τ_id
    prep_opts.J_eq            = opts.I_a;
    prep_opts.friction_params = opts.friction_params;
    opts.prep_opts = prep_opts;
end

%% ========== 三、运行辨识（含可选预处理） ==========
fprintf('===== 一体化流程：预处理(%d) + 摩擦汇总(%d) → 最小参数辨识 =====\n', ...
    use_preprocess_id, load_friction_from_summary);
fprintf('数据: %s\n', csv_file);
fprintf('时间窗: [%.1f, %.1f] s\n', t_start_s, t_end_s);

run_min_param_id_from_csv(csv_file, opts);

fprintf('\n完成。后续验证建议运行：run_full_dynamics_validation（或 run_debug_pi_phys_id_fd_plot 调 π_fd 并画 ID/FD）\n');
