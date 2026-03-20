function T = rmse_struct_to_table(Srmse, tag_col)
% rmse_struct_to_table  将 1x6 RMSE 字段结构体转为表
if nargin < 2, tag_col = 'type'; end
if isempty(Srmse) || ~isstruct(Srmse)
    T = table();
    return;
end
fn = fieldnames(Srmse);
% 仅保留数值字段，避免混入 cell/table 等导致写表失败
is_num = false(numel(fn), 1);
for i = 1:numel(fn)
    is_num(i) = isnumeric(Srmse.(fn{i}));
end
fn = fn(is_num);
nrow = numel(fn);
if nrow == 0
    T = table();
    return;
end
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
    T{i, 2:7} = v;
    T.mean(i) = mean(v, 'omitnan');
    T.max(i)  = max(v, [], 'omitnan');
end
end

