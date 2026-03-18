function [time, q_all] = read_trajectory_csv(csv_file)
% read_trajectory_csv  读取轨迹 CSV（首列时间，其余列为关节位置）
%
% 支持带表头的 CSV：先尝试 readmatrix('NumHeaderLines',1)，失败则 readtable 取 time 与数值列。
% 格式：time, joint1, joint2, ... （12 关节时即 time + 12 列）
%
% 输入: csv_file - CSV 文件路径
% 输出: time - n×1 时间向量 (s)
%       q_all - n×m 关节位置矩阵 (rad)

if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

try
    data = readmatrix(csv_file, 'NumHeaderLines', 1);
    time = data(:, 1);
    q_all = data(:, 2:end);
catch
    T = readtable(csv_file);
    if ismember('time', T.Properties.VariableNames)
        time = T.time;
    else
        time = T.(T.Properties.VariableNames{1});
    end
    q_all = table2array(T(:, 2:end));
end

end
