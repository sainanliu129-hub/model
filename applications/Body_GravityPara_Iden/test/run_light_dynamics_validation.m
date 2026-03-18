%% run_light_dynamics_validation 轻量版“全流程”（脚本）
%
% 目标：实现类似 run_full_dynamics_validation 的能力，但更轻量、更可配置：
%   - 轨迹A：用于辨识/优化（可选）
%   - 轨迹B：用于 ID/FD 对比与绘图（可选，默认用轨迹A或 mat 内轨迹）
%   - 模型集合：cfg.models 决定跑/对比哪些（'cad','beta','pi_rec','pi_fd','pi_phys'）
%
% 依赖：principle/make_avg_data_from_csv.m、identify_min.m、run_identify_pi_fd_only.m、run_plot_id_fd_only.m
%
% 用法示例（直接改下面 cfg 配置块即可）：
%   1) 只做对比（不重新辨识），读 build/id_param_sets.mat：
%      cfg.do_identify = false;
%   2) 轨迹A辨识 + 同轨迹对比：
%      cfg.do_identify = true; cfg.id.csv_file = '...';
%   3) 轨迹A辨识 + 轨迹B对比：
%      cfg.do_identify = true; cfg.id.csv_file='train.csv'; cfg.eval.csv_file='test.csv';

clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
ensure_body_gravity_para_iden_path();

%% 配置（你主要改这里）
cfg = struct();

% 总开关
cfg.do_identify = true;     % true=先跑辨识/优化并落盘；false=直接复用 build/id_param_sets.mat
cfg.do_plot     = true;     % 是否画图
cfg.models      = {'pi_fd','cad','beta'};  % 可选：'cad','beta','pi_rec','pi_fd','pi_phys'

% 轨迹A：辨识/优化用（do_identify=true 时需要）
cfg.id = struct();
cfg.id.csv_file = [];   % 例如 fullfile('..','data','excitation','train.csv')
cfg.id.data_cfg = struct('mode','continuous','use_preprocess',true); % 传给 make_avg_data_from_csv
cfg.id.identify_cfg = struct();  % 额外传给 run_identify_pi_fd_only（比如 opts_fd/n_qdd/n_reg）

% 轨迹B：对比用（为空表示用 build/id_param_sets.mat 内的轨迹）
cfg.eval = struct();
cfg.eval.csv_file = []; % 例如 fullfile('..','data','excitation','test.csv')
cfg.eval.data_cfg = struct('mode','continuous','use_preprocess',true);

% 绘图/统计点数
cfg.N_plot = inf;
cfg.N_fd   = 500;

% 保存开关
save_mat   = true;   % 保存 Out 到 build/light_validation_result.mat
save_excel = true;   % 导出误差汇总到 build/light_validation_summary.xlsx

%% 默认补齐
cfg = set_default(cfg, 'do_identify', true);
cfg = set_default(cfg, 'do_plot', true);
cfg = set_default(cfg, 'models', {'pi_fd','cad','beta'});
cfg = set_default(cfg, 'id', struct());
cfg.id = set_default(cfg.id, 'csv_file', []);
cfg.id = set_default(cfg.id, 'data_cfg', struct());
cfg.id = set_default(cfg.id, 'identify_cfg', struct());
cfg = set_default(cfg, 'eval', struct());
cfg.eval = set_default(cfg.eval, 'csv_file', []);
cfg.eval = set_default(cfg.eval, 'data_cfg', struct());
cfg = set_default(cfg, 'N_plot', inf);
cfg = set_default(cfg, 'N_fd', 500);

build_dir = fullfile(app_root, 'build');
if ~isfolder(build_dir), mkdir(build_dir); end

%% A) 辨识/优化（可选）
if cfg.do_identify
    id_cfg = cfg.id.identify_cfg;
    id_cfg = set_default(id_cfg, 'csv_file', cfg.id.csv_file);
    id_cfg = set_default(id_cfg, 'data_cfg', cfg.id.data_cfg);
    id_cfg = set_default(id_cfg, 'models', cfg.models);
    run_identify_pi_fd_only(id_cfg);
end

%% B) 对比/绘图：可选换轨迹
set_mat = fullfile(build_dir, 'id_param_sets.mat');
if ~isfile(set_mat)
    error('未找到 %s。请先 cfg.do_identify=true 或手动运行 run_identify_pi_fd_only。', set_mat);
end
Sset = load(set_mat);

input_mat_for_plot = set_mat;
if ~isempty(cfg.eval.csv_file)
    eval_cfg = cfg.eval.data_cfg;
    eval_cfg = set_default(eval_cfg, 'limb', Sset.limb);
    [avg_eval, meta_eval] = make_avg_data_from_csv(cfg.eval.csv_file, eval_cfg);

    Sset_eval = Sset;
    Sset_eval.avg_data = avg_eval;
    Sset_eval.meta_eval = meta_eval;
    Sset_eval.eval_csv_file = cfg.eval.csv_file;
    Sset_eval.saved_at_eval = datestr(now);

    input_mat_for_plot = fullfile(build_dir, 'id_param_sets_eval.mat');
    save(input_mat_for_plot, '-struct', 'Sset_eval');
end

plot_cfg = struct();
plot_cfg.input_mat = input_mat_for_plot;
plot_cfg.models = cfg.models;
plot_cfg.N_plot = cfg.N_plot;
plot_cfg.N_fd = cfg.N_fd;
plot_cfg.do_plot = cfg.do_plot;
R = run_plot_id_fd_only(plot_cfg);

Out = struct();
Out.cfg = cfg;
Out.input_mat_for_plot = input_mat_for_plot;
Out.metrics = R;
Out.saved_at = datestr(now);

%% C) 保存（mat + excel）
if save_mat
    out_mat = fullfile(build_dir, 'light_validation_result.mat');
    save(out_mat, 'Out');
    fprintf('已保存 mat: %s\n', out_mat);
end

if save_excel
    out_xlsx = fullfile(build_dir, 'light_validation_summary.xlsx');

    % ID RMSE 表
    T_id = rmse_struct_to_table(R.id.rmse, 'id_rmse');
    % FD RMSE 表
    T_fd = rmse_struct_to_table(R.fd.rmse, 'fd_rmse');

    writetable(T_id, out_xlsx, 'Sheet', 'ID_RMSE');
    writetable(T_fd, out_xlsx, 'Sheet', 'FD_RMSE');
    fprintf('已导出 Excel: %s\n', out_xlsx);
end

%% local functions（脚本尾部）
function s = set_default(s, name, val)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = val;
    end
end

function T = rmse_struct_to_table(Srmse, tag_col)
    if nargin < 2, tag_col = 'type'; end
    if isempty(Srmse) || ~isstruct(Srmse)
        T = table();
        return;
    end
    fn = fieldnames(Srmse);
    nrow = numel(fn);
    T = table('Size', [nrow, 9], ...
        'VariableTypes', {'string','double','double','double','double','double','double','double','double'}, ...
        'VariableNames', {tag_col,'j1','j2','j3','j4','j5','j6','mean','max'});
    for i = 1:nrow
        k = fn{i};
        v = Srmse.(k);
        v = v(:).';
        if numel(v) < 6
            v = [v, nan(1, 6-numel(v))];
        else
            v = v(1:6);
        end
        T.(tag_col)(i) = string(k);
        T{i, 2:7} = num2cell(v);
        T.mean(i) = mean(v, 'omitnan');
        T.max(i)  = max(v);
    end
end

