%% example_run_min_param_id  调用最小参数辨识流程（一键运行）
%
% 默认：连续窗模式，只取中间激励段（如 3–7s）+ qd 50Hz 低通 + qdd SG → 辨识。
% 前后过渡段信息密度低，会拉差 W_min 条件数，故建议只取激励段。
%
% 使用步骤：
%   1. 将激励采集 CSV 放到 data/excitation/（或 data/跑步/）
%   2. 按你的轨迹修改 t_start_s / t_end_s（激励段时间范围）
%   3. 运行本脚本（F5）
%
% 【对原始数据不进行任何处理】设置：
%   设 use_preprocess_id = false（不做 SG 求导、不做 τ 低通、不扣摩擦/转子），
%   且 continuous_opts 里 qd_lowpass_fc_Hz=0、qdd_smooth_half=0（窗内不滤波、qdd 仅由 qd 中心差分）。
%   可选：t_start_s/t_end_s 设为 [] 表示不裁时间、用全段。
%   见下方 "raw_data_no_processing" 配置块。

clear;
clc;
close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();
csv_file = fullfile(app_root, 'data', 'excitation', 'PD-M1-v0_multi_joint_20260305-195432_exctra_real.csv');
% csv_file = fullfile(app_root, 'data', '跑步', 'PD-M1-v0_multi_joint_20260226-182548.csv');

% 是否对原始数据不进行任何处理（仅时间窗 + 窗内 qd→qdd 中心差分，无 SG/τ 滤波/摩擦补偿）
raw_data_no_processing = true;   % true = 不预处理、窗内不滤波；false = 可用预处理（见 run_min_param_id_from_csv 的 use_preprocess_id/prep_opts）

if raw_data_no_processing
    % 不预处理：不做 SG、不做 τ 低通、不扣摩擦/转子；窗内 qd 不低通、qdd 不平滑（仅中心差分）
    continuous_opts = struct('t_start_s', 2.1, 't_end_s', 4.1, ...
        'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
    opts = struct('mode', 'continuous', 'continuous_opts', continuous_opts, ...
        'use_preprocess_id', false);
    % 若要用全段时间不裁：continuous_opts.t_start_s = []; continuous_opts.t_end_s = [];
else
    % 与 test_ReMatrix_E1_limb_URDF 一致：窗 [2,4] s，窗内可不滤波
    t_start_s = 2.1;
    t_end_s   = 4.1;
    continuous_opts = struct('t_start_s', t_start_s, 't_end_s', t_end_s, ...
        'qd_lowpass_fc_Hz', 0, 'qdd_smooth_half', 0);
    opts = struct('mode', 'continuous', 'continuous_opts', continuous_opts);
end

run_min_param_id_from_csv(csv_file, opts);
