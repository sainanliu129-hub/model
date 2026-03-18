function data_out = read_python_upper_scan_csv(file_or_list)
% read_python_upper_scan_csv  读取 Python 上位机导出的 6/7 列 CSV，输出 [t, current_A, speed_rad/s]（仅在此处做 rpm→rad/s 转换）
%
% 支持输入：
%   - 单个文件路径 char/string
%   - cell/string 数组：多个 CSV 依次拼接
%
% CSV 典型格式（见 applications/Friction_Iden/Python上位机电机扫描说明.md）：
%   6 列：timestamp_s, direction, target_speed_rpm, actual_speed_rpm, phase_current_A, temp_degC
%   7 列（电流扫描）：timestamp_s, direction, target_speed_rpm, command_current_A, actual_speed_rpm, phase_current_A, temp_degC
%   7 列（加速度扫描）：timestamp_s, direction, accel_rpm_s, target_speed_rpm, actual_speed_rpm, phase_current_A, temp_degC
%
% 输出：
%   data_out: N×3 double，列分别为 [t_s, current_A, speed_rad/s]（速度带正负，仅在此处做一次 rpm→rad/s）
% 使用的电流列为 phase_current_A（实际相电流），不是 command_current_A（指令电流）。

if nargin < 1
    error('read_python_upper_scan_csv:MissingInput', '需要提供 CSV 文件路径或列表');
end

file_list = file_or_list;
if isstring(file_list), file_list = cellstr(file_list); end
if ischar(file_list) || isstring(file_list)
    file_list = {char(file_list)};
end
if ~iscell(file_list) || isempty(file_list)
    error('read_python_upper_scan_csv:BadInput', '输入必须是文件路径或 cell 文件列表');
end

data_all = [];
for i = 1:numel(file_list)
    f = file_list{i};
    if ~exist(f, 'file')
        error('read_python_upper_scan_csv:FileNotFound', '文件不存在: %s', f);
    end

    raw = readcell(f, 'Delimiter', ',');
    if isempty(raw) || size(raw,1) < 2
        continue;
    end

    % 如果第一行是表头（包含 timestamp 等字符串），跳过
    if ischar(raw{1,1}) || isstring(raw{1,1})
        s = string(raw{1,1});
        if contains(lower(s), "timestamp")
            raw = raw(2:end, :);
        end
    end

    ncol = size(raw, 2);
    if ncol < 6
        error('read_python_upper_scan_csv:BadFormat', '列数不足（期望 6 或 7 列），实际 %d 列: %s', ncol, f);
    end

    % 兼容 6/7 列：列位置固定
    t_col = 1;
    if ncol == 6
        speed_col = 4;   % actual_speed_rpm
        cur_col = 5;    % phase_current_A（实际电流）
    else
        % 7 列：actual_speed 在第 5 列，phase_current_A（实际电流）在第 6 列；不取 command_current_A
        speed_col = 5;
        cur_col = 6;
    end

    t = col_to_double(raw(:, t_col));
    speed_rpm = col_to_double(raw(:, speed_col));
    current_a = col_to_double(raw(:, cur_col));
    % 速度带正负：根据 direction（第 2 列）将反向设为负，正向/未知保持原符号或取正
    if ncol >= 2
        for row = 1:size(raw, 1)
            d = raw{row, 2};
            if ischar(d) || isstring(d)
                s = lower(string(d));
                % 负向：neg / reverse / "-" / negative 等
                if contains(s, "neg") || contains(s, "reverse") || strip(s) == "-"
                    speed_rpm(row) = -abs(speed_rpm(row));
                else
                    speed_rpm(row) = abs(speed_rpm(row));
                end
            end
            % 若 direction 非字符串（空或数字），不修改该行速度，保留 CSV 原有正负
        end
    end

    % 仅在此处做一次 rpm → rad/s，后续全用 rad/s
    speed_rads = speed_rpm * (2*pi/60);
    % 去掉 NaN 行
    valid = ~(isnan(t) | isnan(speed_rads) | isnan(current_a));
    t = t(valid);
    speed_rads = speed_rads(valid);
    current_a = current_a(valid);

    data_all = [data_all; [t(:), current_a(:), speed_rads(:)]]; %#ok<AGROW>
end

if isempty(data_all)
    data_out = zeros(0, 3);
else
    data_out = data_all;
end

end

function x = col_to_double(col)
% 将 readcell 读出的任意类型列转换为 double
x = nan(size(col));
for k = 1:numel(col)
    v = col{k};
    if isempty(v)
        x(k) = NaN;
    elseif isnumeric(v)
        x(k) = double(v);
    elseif islogical(v)
        x(k) = double(v);
    else
        xs = strtrim(string(v));
        if strlength(xs) == 0
            x(k) = NaN;
        else
            x(k) = str2double(xs);
        end
    end
end
end
