function run_batch_fd_summary(opts)
% run_batch_fd_summary  复用 run_plot_id_fd_only 生成 FD 汇总（薄封装）

if nargin < 1, opts = struct(); end
app_root = fullfile(fileparts(mfilename('fullpath')), '..');

cfg = struct();
cfg.input_mat = get_field_or(opts, 'input_mat', fullfile(app_root, 'build', 'id_param_sets.mat'));
cfg.models = get_field_or(opts, 'models', {'cad','beta','pi_rec','pi_phys','pi_fd'});
cfg.N_plot = 1;            % ID 统计交给 run_batch_id_summary
cfg.N_fd = get_field_or(opts, 'N_fd', inf);
cfg.do_plot = get_field_or(opts, 'do_plot', false);
cfg.save_excel = false;

R = run_plot_id_fd_only(cfg);
out_xlsx = fullfile(app_root, 'build', 'fd_batch_summary.xlsx');
meta_tbl = table(string(cfg.input_mat), cfg.N_fd, 'VariableNames', {'input_mat','N_fd'});
export_validation_summary_excel(out_xlsx, struct(), R.fd.rmse, meta_tbl);
fprintf('[FD] 批量汇总完成: %s\n', out_xlsx);
end

function v = get_field_or(s, name, default_v)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default_v; end
end

