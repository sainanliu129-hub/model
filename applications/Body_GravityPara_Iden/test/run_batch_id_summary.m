function run_batch_id_summary(opts)
% run_batch_id_summary  复用 run_plot_id_fd_only 生成 ID 汇总（薄封装）

if nargin < 1, opts = struct(); end
app_root = fullfile(fileparts(mfilename('fullpath')), '..');

cfg = struct();
cfg.input_mat = get_field_or(opts, 'input_mat', fullfile(app_root, 'build', 'id_param_sets.mat'));
cfg.models = get_field_or(opts, 'models', {'cad','beta','pi_rec','pi_phys','pi_fd'});
cfg.N_plot = get_field_or(opts, 'N_plot', inf);
cfg.N_fd = 1;              % FD 统计交给 run_batch_fd_summary
cfg.do_plot = get_field_or(opts, 'do_plot', false);
cfg.save_excel = false;

R = run_plot_id_fd_only(cfg);
out_xlsx = fullfile(app_root, 'build', 'id_batch_summary.xlsx');
meta_tbl = table(string(cfg.input_mat), 'VariableNames', {'input_mat'});
export_validation_summary_excel(out_xlsx, R.id.rmse, struct(), meta_tbl);
fprintf('[ID] 批量汇总完成: %s\n', out_xlsx);
end

function v = get_field_or(s, name, default_v)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default_v; end
end

