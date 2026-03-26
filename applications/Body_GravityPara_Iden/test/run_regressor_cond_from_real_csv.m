% run_regressor_cond_from_real_csv
% 直接运行：用“实机得到的 csv”计算辨识回归矩阵条件数（cond(Y_full / Y_min / W_min)）
%
% 使用方式：
%   1) 默认：自动选 data/excitation/ 下最新 csv
%   2) 你也可以手动指定 csv_file
%   3) limb 建议 'left_leg' 或 'right_leg'
%
% 输出：
%   控制台打印 cond(Y_full), cond(Y_min), cond(W_min) 以及近零列信息（如果有）

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath')); %#ok<NASGU>
repo_root = fileparts(fileparts(fileparts(this_dir))); % model_e1
addpath(genpath(repo_root));

% -------- User config --------
% 手动指定时写成相对路径（相对 Body_GravityPara_Iden/app_root）
% 例如：csv_file = 'data/excitation/xxx.csv';
csv_file = 'data/excitation/PD-M1-v0_multi_joint_20260325-183300.csv';      % 空则自动选最新
limb = 'left_leg';  % 'left_leg' | 'right_leg'
para_order = 1;     % 与 identify_min / test_ReMatrix 的一致性：默认 1

% 差分 qd->qdd 噪声会抬高条件数；若 csv 含 acc/qdd 列可设 qd_lowpass_fc_Hz=0
qd_lowpass_fc_Hz = 25;   % 0 表示不对 qd 进行低通
skip = 5;               % 下采样步长（越大越快）
do_plot = true;         % 不弹回归能量条形图（更专注对比图）
do_vel_acc_compare = true; % 画期望速度(qd_ref) vs 实际速度/加速度(qd/qdd)
t_start_s = 2.0;         % 可选：只使用从该时刻到 t_end_s 的数据（单位：s）
t_end_s = 12.0;           % 可选：只使用到该时刻（单位：s）

opts = struct();
opts.skip = skip;
opts.qd_lowpass_fc_Hz = qd_lowpass_fc_Hz;
opts.do_plot = do_plot;
opts.do_vel_acc_compare = do_vel_acc_compare;
opts.ref_mat_path = '';
opts.t_start_s = t_start_s;
opts.t_end_s = t_end_s;

% -------- Auto select latest csv --------
if isempty(csv_file)
    app_root = fullfile(repo_root, 'applications', 'Body_GravityPara_Iden');
    dir1 = fullfile(app_root, 'data', 'excitation');
    dir2 = fullfile(app_root, 'data', '跑步');

    csv_candidates = {};
    if isfolder(dir1)
        f = dir(fullfile(dir1, '*.csv'));
        for i = 1:numel(f)
            csv_candidates{end+1} = fullfile(f(i).folder, f(i).name); %#ok<SAGROW>
        end
    end
    if isfolder(dir2)
        f = dir(fullfile(dir2, '*.csv'));
        for i = 1:numel(f)
            csv_candidates{end+1} = fullfile(f(i).folder, f(i).name); %#ok<SAGROW>
        end
    end

    if isempty(csv_candidates)
        error('未找到任何 CSV：请把 csv 放到 applications/Body_GravityPara_Iden/data/excitation 或 data/跑步 下。');
    end

    % 选最新 datenum
    dn = zeros(numel(csv_candidates),1);
    for i = 1:numel(csv_candidates)
        dn(i) = dir(csv_candidates{i});
        if isstruct(dn(i))
            % 理论不会走到这里，但保底
            dn(i) = now;
        end
    end
    % dir返回结构体时 dn(i) 上面会出问题，改用日期排序
    info = cell(numel(csv_candidates),1);
    for i = 1:numel(csv_candidates)
        info{i} = dir(csv_candidates{i});
    end
    [~, idx] = sort(cellfun(@(s) s.datenum, info), 'descend');
    csv_file_abs = csv_candidates{idx(1)};
else
    % 相对路径：相对 app_root
    app_root = fullfile(repo_root, 'applications', 'Body_GravityPara_Iden');
    if exist(fullfile(app_root, csv_file), 'file')
        csv_file_abs = fullfile(app_root, csv_file);
    else
        % 也允许用户直接填绝对路径
        csv_file_abs = csv_file;
    end
end

fprintf('Using CSV: %s\n', csv_file_abs);
results = compute_regressor_cond_from_real_csv(csv_file_abs, limb, para_order, opts);

% 结构体结果也可以继续被上层脚本使用
assignin('base', 'regressor_cond_results', results);

