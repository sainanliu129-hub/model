function [t_out, q_out, qd_out, qdd_out, tau_out, aux, prep_used] = run_id_preprocess_pipeline(t, q, qd, tau, prep_opts)
% run_id_preprocess_pipeline  预处理统一入口：低通滤波 + 可选补偿（解耦辨识主流程）
%
% 目标：
% - 把“滤波 + 补偿”集中在预处理函数，不混在辨识代码里。
% - 补偿可选：do_compensation=false 时仅做低通预处理，不做 J_eq/摩擦/偏置补偿。
%
% 关键选项（prep_opts）：
%   基础滤波：
%     t_start_s, t_end_s, q_lowpass_fc_Hz, q_lowpass_order,
%     tau_lowpass_fc_Hz, tau_lowpass_order, do_plot
%
%   补偿开关与来源：
%     do_compensation            - true/false，默认 false
%     J_eq, friction_params, tau_bias
%     load_friction_from_summary - true 时从 Friction_Iden 汇总表加载
%     n_joints, row_for_joint, friction_iden_dir
%
% 输出：
%   tau_out 为 preprocess_id_data 输出的 tau_id（若 do_compensation=false，则 tau_id≈tau_s）

if nargin < 5
    prep_opts = struct();
end

prep = prep_opts;
if ~isfield(prep, 't_start_s'), prep.t_start_s = 2.1; end
if ~isfield(prep, 't_end_s'), prep.t_end_s = 4.1; end
if ~isfield(prep, 'q_lowpass_fc_Hz'), prep.q_lowpass_fc_Hz = 25; end
if ~isfield(prep, 'q_lowpass_order'), prep.q_lowpass_order = 2; end
if ~isfield(prep, 'tau_lowpass_fc_Hz'), prep.tau_lowpass_fc_Hz = 25; end
if ~isfield(prep, 'tau_lowpass_order'), prep.tau_lowpass_order = 2; end
if ~isfield(prep, 'do_plot'), prep.do_plot = false; end

if ~isfield(prep, 'do_compensation'), prep.do_compensation = false; end
if ~isfield(prep, 'load_friction_from_summary'), prep.load_friction_from_summary = false; end
if ~isfield(prep, 'n_joints'), prep.n_joints = 6; end
if ~isfield(prep, 'row_for_joint'), prep.row_for_joint = []; end
if ~isfield(prep, 'friction_iden_dir'), prep.friction_iden_dir = ''; end

if prep.do_compensation
    if prep.load_friction_from_summary
        friction_iden_dir = prep.friction_iden_dir;
        if isempty(friction_iden_dir)
            app_root = fileparts(fileparts(mfilename('fullpath')));
            friction_iden_dir = fullfile(app_root, '..', 'Friction_Iden');
        end
        if isfolder(friction_iden_dir) && isempty(which('load_friction_stribeck_from_summary'))
            addpath(friction_iden_dir);
        end
        load_opts = struct();
        if ~isempty(prep.row_for_joint), load_opts.row_for_joint = prep.row_for_joint; end
        [fp_loaded, Ia_loaded] = load_friction_stribeck_from_summary(prep.n_joints, load_opts);
        if ~isfield(prep, 'friction_params') || isempty(prep.friction_params)
            prep.friction_params = fp_loaded;
        end
        if ~isfield(prep, 'J_eq') || isempty(prep.J_eq)
            prep.J_eq = Ia_loaded;
        end
    end
else
    % 禁用补偿时，强制清空补偿项，确保只做滤波
    prep.J_eq = [];
    prep.friction_params = [];
    prep.tau_bias = [];
end

[t_out, q_out, qd_out, qdd_out, tau_out, aux] = preprocess_id_data(t, q, qd, tau, prep);
prep_used = prep;
end

