function rows_out = append_result_summary(summary_file, result_struct)
% append_result_summary  追加一行方案结果到 summary（.mat + .csv）
%
% 目的：
%   - 每跑完一组 scheme，把关键结果汇总成一行
%   - .mat：完整保存 result_struct（可包含 pi_* 等向量/矩阵）
%   - .csv：只写标量/短字段（并自动写 pi_* 的 norm/n，便于筛选）
%
% 用法：
%   append_result_summary('/path/summary_scheme', result_struct)
%   % 自动生成：
%   %   /path/summary_scheme.mat
%   %   /path/summary_scheme.csv
%
% 约定：
%   - result_struct 建议至少包含：scheme_name/use_friction_56/use_mix_params/feasible/...
%   - 若 result_struct 里包含 pi_cad/pi_rec/pi_phys/pi_fd 等数组，
%     本函数会在 .csv 里额外写 pi_*_norm 与 pi_*_n（不写完整数组）。

if nargin < 2
    error('append_result_summary: 需要参数 summary_file 和 result_struct');
end
if ~isstruct(result_struct)
    error('append_result_summary: result_struct 必须是 struct');
end

% ---------- base path ----------
[base_dir, base_name, ext] = fileparts(summary_file);
if isempty(ext)
    base_path = fullfile(base_dir, base_name);
elseif strcmpi(ext, '.mat') || strcmpi(ext, '.csv')
    base_path = fullfile(base_dir, base_name);
else
    % 允许用户传类似 xxx.txt：仍然按 base 处理为 .mat/.csv
    base_path = fullfile(base_dir, [base_name]);
end

mat_path = [base_path '.mat'];
csv_path = [base_path '.csv'];

% ---------- auto enrich ----------
piFields = {'pi_cad','pi_rec','pi_phys','pi_fd'};
for i = 1:numel(piFields)
    fn = piFields{i};
    if isfield(result_struct, fn)
        v = result_struct.(fn);
        if isnumeric(v) || islogical(v)
            result_struct.([fn '_norm']) = norm(v(:));
            result_struct.([fn '_n']) = numel(v);
        end
    end
end

if ~isfield(result_struct, 'timestamp') || isempty(result_struct.timestamp)
    result_struct.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

if ~isfield(result_struct, 'scheme_name') || isempty(result_struct.scheme_name)
    result_struct.scheme_name = '';
end

% ---------- load existing ----------
rows = struct();
if isfile(mat_path)
    S = load(mat_path);
    if isfield(S, 'rows')
        rows = S.rows;
    elseif isfield(S, 'result_rows')
        rows = S.result_rows;
    end
end

if isempty(rows)
    rows = result_struct;
else
    % 不同版本脚本可能写入了不同字段；先做字段并集对齐再拼接
    [rows_aligned, new_aligned] = align_struct_fields(rows, result_struct);
    rows = [rows_aligned, new_aligned]; %#ok<AGROW>
end

% ---------- save mat ----------
save(mat_path, 'rows');

% ---------- build csv table ----------
nrows = numel(rows);

% union fields across rows
all_fields = {};
for i = 1:nrows
    f = fieldnames(rows(i));
    all_fields = [all_fields; f(:)]; %#ok<AGROW>
end
all_fields = unique(all_fields, 'stable');

% preferred order
preferred_order = { ...
    'scheme_name', 'use_friction_56', 'use_mix_params', 'feasible', ...
    'id_rmse', 'fd_qdd_rmse', 'fd_qd_rmse_h5', 'fd_qd_rmse_full', 'fd_q_rmse_full', ...
    'mass_minEig_min', 'mass_cond_med', 'notes' ...
};

ordered_fields = {};
for i = 1:numel(preferred_order)
    if any(strcmp(all_fields, preferred_order{i}))
        ordered_fields{end+1,1} = preferred_order{i}; %#ok<AGROW>
    end
end
% append the rest
csv_exclude_fields = {'pi_cad','pi_rec','pi_phys','pi_fd'};
for i = 1:numel(all_fields)
    if ~any(strcmp(ordered_fields, all_fields{i})) && ~any(strcmp(csv_exclude_fields, all_fields{i}))
        ordered_fields{end+1,1} = all_fields{i}; %#ok<AGROW>
    end
end

% sanitize field names for table variable names
var_names = cell(size(ordered_fields));
for i = 1:numel(ordered_fields)
    var_names{i} = matlab.lang.makeValidName(ordered_fields{i});
end

T = table();
for fi = 1:numel(ordered_fields)
    col_name = ordered_fields{fi};
    vname = var_names{fi};

    col_num = nan(nrows, 1);
    col_isnum = true;
    col_cell = cell(nrows, 1);

    for ri = 1:nrows
        if isfield(rows(ri), col_name)
            v = rows(ri).(col_name);
        else
            v = [];
        end

        if isempty(v)
            col_num(ri) = NaN;
            col_cell{ri} = '';
            continue;
        end

        if islogical(v) && isscalar(v)
            col_num(ri) = double(v);
            col_cell{ri} = '';
        elseif isnumeric(v) && isscalar(v)
            col_num(ri) = double(v);
            col_cell{ri} = '';
        elseif (ischar(v) || isstring(v)) && isscalar(v)
            col_isnum = false;
            col_cell{ri} = char(string(v));
            col_num(ri) = NaN;
        else
            % non-scalar numeric/array: convert to short string
            col_isnum = false;
            try
                col_cell{ri} = mat2str(v(1:min(numel(v), 6)));
            catch
                col_cell{ri} = '';
            end
            col_num(ri) = NaN;
        end
    end

    if col_isnum
        T.(vname) = col_num;
    else
        % use cellstr column for mixed types
        col_cell = cellfun(@(c) char(string(c)), col_cell, 'UniformOutput', false);
        T.(vname) = col_cell;
    end
end

writetable(T, csv_path);

rows_out = rows;
end

function [A2, B2] = align_struct_fields(A, B)
% align_struct_fields 让结构体数组 A 与结构体 B 具有相同字段集合
    if isempty(A)
        A2 = A; B2 = B; return;
    end
    fa = fieldnames(A);
    fb = fieldnames(B);
    allf = unique([fa; fb], 'stable');

    A2 = A;
    B2 = B;
    for i = 1:numel(allf)
        fn = allf{i};
        if ~isfield(A2, fn)
            [A2.(fn)] = deal([]);
        end
        if ~isfield(B2, fn)
            B2.(fn) = [];
        end
    end
end

