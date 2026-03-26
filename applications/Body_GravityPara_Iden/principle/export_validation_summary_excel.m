function export_validation_summary_excel(out_xlsx, id_rmse, fd_rmse, extra_table, opts)
% export_validation_summary_excel  导出验证汇总到 Excel（复用接口）
%
% 输入：
%   out_xlsx   - 输出路径
%   id_rmse    - struct，每个字段为 1x6 RMSE（可空）
%   fd_rmse    - struct，每个字段为 1x6 RMSE（可空）
%   extra_table- 可选 table，写到 Meta 工作表
%   opts       - 可选 struct:
%                .id_maxerr     struct，每个字段为 1x6 Max|err|
%                .fd_maxerr     struct，每个字段为 1x6 Max|err|
%                .id_plot_files table，列建议为 id_compare_png/id_error_png
%                .fd_plot_files table，列建议为 fd_compare_png/fd_error_png；
%                  可选 fd_q_compare_png/fd_q_error_png（关节角 q 对比/误差）

if nargin < 4, extra_table = table(); end
if nargin < 5 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'embed_figures'), opts.embed_figures = true; end
if ~isfield(opts, 'write_plot_path_sheets'), opts.write_plot_path_sheets = false; end
out_dir = fileparts(out_xlsx);
if ~isempty(out_dir) && ~isfolder(out_dir)
    mkdir(out_dir);
end

if ~isempty(id_rmse) && ~isempty(fieldnames(id_rmse))
    T_id = rmse_struct_to_table(id_rmse, 'id_rmse');
    writetable(T_id, out_xlsx, 'Sheet', 'ID_RMSE');
end
if ~isempty(fd_rmse) && ~isempty(fieldnames(fd_rmse))
    T_fd = rmse_struct_to_table(fd_rmse, 'fd_rmse');
    writetable(T_fd, out_xlsx, 'Sheet', 'FD_RMSE');
end
if isfield(opts, 'id_maxerr') && ~isempty(opts.id_maxerr) && ~isempty(fieldnames(opts.id_maxerr))
    T_id_max = rmse_struct_to_table(opts.id_maxerr, 'id_maxerr');
    writetable(T_id_max, out_xlsx, 'Sheet', 'ID_MAXERR');
end
if isfield(opts, 'fd_maxerr') && ~isempty(opts.fd_maxerr) && ~isempty(fieldnames(opts.fd_maxerr))
    T_fd_max = rmse_struct_to_table(opts.fd_maxerr, 'fd_maxerr');
    writetable(T_fd_max, out_xlsx, 'Sheet', 'FD_MAXERR');
end
if ~isempty(extra_table) && istable(extra_table)
    writetable(extra_table, out_xlsx, 'Sheet', 'Meta');
end
if isfield(opts, 'mass_health_table') && istable(opts.mass_health_table) && ~isempty(opts.mass_health_table)
    writetable(opts.mass_health_table, out_xlsx, 'Sheet', 'MASS_MATRIX_HEALTH');
end
if isfield(opts, 'dyn_params_table') && istable(opts.dyn_params_table) && ~isempty(opts.dyn_params_table)
    writetable(opts.dyn_params_table, out_xlsx, 'Sheet', 'DYN_PARAMS');
end
if opts.write_plot_path_sheets && isfield(opts, 'id_plot_files') && ~isempty(opts.id_plot_files) && istable(opts.id_plot_files)
    writetable(opts.id_plot_files, out_xlsx, 'Sheet', 'ID_PLOTS');
end
if opts.write_plot_path_sheets && isfield(opts, 'fd_plot_files') && ~isempty(opts.fd_plot_files) && istable(opts.fd_plot_files)
    writetable(opts.fd_plot_files, out_xlsx, 'Sheet', 'FD_PLOTS');
end

if opts.embed_figures
    ok = embed_validation_figures_with_python(out_xlsx, opts);
    if ok
        fprintf('已将图嵌入 Excel 对应工作表。\n');
    else
        fprintf('未完成 Excel 嵌图（请安装: pip install openpyxl Pillow）。\n');
    end
end
fprintf('已导出验证汇总: %s\n', out_xlsx);
end

function ok = embed_validation_figures_with_python(xlsx_path, opts)
script_dir = fileparts(mfilename('fullpath'));
py_script = fullfile(script_dir, 'embed_validation_figures_xlsx.py');
if exist(py_script, 'file') ~= 2
    ok = false;
    return;
end

id_compare = '';
id_error = '';
fd_compare = '';
fd_error = '';
fd_q_compare = '';
fd_q_error = '';

if isfield(opts, 'id_plot_files') && istable(opts.id_plot_files) && ~isempty(opts.id_plot_files)
    if any(strcmp(opts.id_plot_files.Properties.VariableNames, 'id_compare_png'))
        id_compare = char(string(opts.id_plot_files.id_compare_png(1)));
    end
    if any(strcmp(opts.id_plot_files.Properties.VariableNames, 'id_error_png'))
        id_error = char(string(opts.id_plot_files.id_error_png(1)));
    end
end
if isfield(opts, 'fd_plot_files') && istable(opts.fd_plot_files) && ~isempty(opts.fd_plot_files)
    if any(strcmp(opts.fd_plot_files.Properties.VariableNames, 'fd_compare_png'))
        fd_compare = char(string(opts.fd_plot_files.fd_compare_png(1)));
    end
    if any(strcmp(opts.fd_plot_files.Properties.VariableNames, 'fd_error_png'))
        fd_error = char(string(opts.fd_plot_files.fd_error_png(1)));
    end
    if any(strcmp(opts.fd_plot_files.Properties.VariableNames, 'fd_q_compare_png'))
        fd_q_compare = char(string(opts.fd_plot_files.fd_q_compare_png(1)));
    end
    if any(strcmp(opts.fd_plot_files.Properties.VariableNames, 'fd_q_error_png'))
        fd_q_error = char(string(opts.fd_plot_files.fd_q_error_png(1)));
    end
end

cmd = sprintf('python3 "%s" "%s" "%s" "%s" "%s" "%s" "%s" "%s" 2>&1', ...
    py_script, xlsx_path, id_compare, id_error, fd_compare, fd_error, fd_q_compare, fd_q_error);
[st, msg] = system(cmd);
ok = (st == 0);
if ~ok && ~isempty(msg)
    fprintf('  Python 嵌图: %s\n', strrep(msg, newline, [newline '  ']));
end
end

