%% run_split_biped_diagnosis_csv  双腿诊断 CSV → 两条单腿名义 CSV（先预览再写）
%
% 假定 CSV 里记录的关节轨迹为**期望轨迹**（指令/规划），不是编码器实测反馈。
% 若期望角只写在 cmd_*、pos_* 未与之一致，在 cfg 里设 use_cmd_for_desired = true。
%
% 静止腿与激励脚本一致：整段**固定**为 other_leg_safe_6。
%   左腿动 → 右腿 [0, -0.436, 0, 0, 0, 0]；右腿动 → 左腿 [0, 0.436, 0, 0, 0, 0]
% 另输出静止腿全零版本。仅需一个输入 csv_in。
% 写出格式与激励 CSV 相同：time + leg_l1_joint..leg_r6_joint（见 write_leg_trajectory_csv_plan_format）。

clc;clear;close;

script_dir = fileparts(mfilename('fullpath'));
app_root = fullfile(script_dir, '..');
repo_root = fileparts(fileparts(app_root));
addpath(fullfile(repo_root, 'utility_function'));
addpath(app_root);

%% ---------- 修改以下路径 ----------
cfg = struct();
cfg.csv_in = fullfile(app_root, 'data', 'excitation', '0-260116_run_softly_轻跑步_002_leg_500Hz_PD-M1-v0_multi_joint_20260325-035526.csv');
cfg.active_limb = 'left_leg';
cfg.out_dir = '';
cfg.use_cmd_for_desired = true;  % true：读入后 pos := cmd（期望只在 cmd 列时）
% cfg.still_leg_q = [0, -0.436, 0, 0, 0, 0];  % 可选，覆盖默认 safe 6 维
%% --------------------------------

if ~isfile(cfg.csv_in)
    error('输入不存在: %s', cfg.csv_in);
end

args = {'active_limb', cfg.active_limb, 'out_dir', cfg.out_dir, ...
    'use_cmd_for_desired', cfg.use_cmd_for_desired, 'do_plot', true};
if isfield(cfg, 'still_leg_q') && ~isempty(cfg.still_leg_q)
    args = [args, {'still_leg_q', cfg.still_leg_q}];
end

out_paths = split_biped_csv_single_leg_static(cfg.csv_in, args{:});

fprintf('已写出:\n  %s\n  %s\n', out_paths.safe, out_paths.zero);
