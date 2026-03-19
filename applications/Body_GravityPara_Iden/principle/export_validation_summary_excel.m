function export_validation_summary_excel(out_xlsx, id_rmse, fd_rmse, extra_table)
% export_validation_summary_excel  导出验证汇总到 Excel（复用接口）
%
% 输入：
%   out_xlsx   - 输出路径
%   id_rmse    - struct，每个字段为 1x6 RMSE（可空）
%   fd_rmse    - struct，每个字段为 1x6 RMSE（可空）
%   extra_table- 可选 table，写到 Meta 工作表

if nargin < 4, extra_table = table(); end
out_dir = fileparts(out_xlsx);
if ~isempty(out_dir) && ~isfolder(out_dir)
    mkdir(out_dir);
end

if ~isempty(id_rmse)
    T_id = rmse_struct_to_table(id_rmse, 'id_rmse');
    writetable(T_id, out_xlsx, 'Sheet', 'ID_RMSE');
end
if ~isempty(fd_rmse)
    T_fd = rmse_struct_to_table(fd_rmse, 'fd_rmse');
    writetable(T_fd, out_xlsx, 'Sheet', 'FD_RMSE');
end
if ~isempty(extra_table) && istable(extra_table)
    writetable(extra_table, out_xlsx, 'Sheet', 'Meta');
end
fprintf('已导出验证汇总: %s\n', out_xlsx);
end

