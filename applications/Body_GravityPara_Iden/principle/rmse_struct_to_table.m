function T = rmse_struct_to_table(Srmse, tag_col)
% rmse_struct_to_table  将 1x6 RMSE 字段结构体转为表
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

