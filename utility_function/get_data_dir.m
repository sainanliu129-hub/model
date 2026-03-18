function data_dir = get_data_dir(subdir)
% get_data_dir  返回数据文件根目录 data 的完整路径（可选子目录）
%
% 用法:
%   data_dir = get_data_dir()           % 返回 .../data
%   data_dir = get_data_dir('dynamics') % 返回 .../data/dynamics
%
% 若目录不存在会自动创建。建议所有脚本读取的输入数据（.mat/.csv/.txt 等）默认从此目录查找。

if nargin < 1
    subdir = '';
end

% 以“调用当前函数的代码所在目录”为起点，向上查找最近的 data/ 目录
st = dbstack('-completenames');
if numel(st) >= 2 && isfield(st(2), 'file') && ~isempty(st(2).file)
    base_dir = fileparts(st(2).file);
else
    base_dir = pwd;
end

root_dir = base_dir;
found = false;
for k = 1:20
    if exist(fullfile(root_dir, 'data'), 'dir')
        found = true;
        break;
    end
    parent_dir = fileparts(root_dir);
    if isempty(parent_dir) || strcmp(parent_dir, root_dir)
        break;
    end
    root_dir = parent_dir;
end

% 如果没找到 data/，则退回到 base_dir 下创建
if ~found
    root_dir = base_dir;
end

data_dir = fullfile(root_dir, 'data', subdir);
if ~isempty(subdir)
    data_dir = strrep(data_dir, [filesep filesep], filesep);
end
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
end
