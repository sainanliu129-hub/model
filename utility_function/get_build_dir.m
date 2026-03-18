function build_dir = get_build_dir(subdir)
% get_build_dir  返回项目生成文件输出根目录 build 的完整路径（可选子目录）
%
% 用法:
%   build_dir = get_build_dir()           % 返回 .../matlab/build
%   build_dir = get_build_dir('plan')     % 返回 .../matlab/build/plan
%
% 若目录不存在会自动创建。建议所有脚本生成的 .mat/.csv/.txt 等默认写入此目录。

if nargin < 1
    subdir = '';
end
% 本文件在 utility_function 下，其上级即为 matlab 根
matlab_root = fileparts(fileparts(mfilename('fullpath')));
build_dir = fullfile(matlab_root, 'build', subdir);
if ~isempty(subdir)
    build_dir = strrep(build_dir, [filesep filesep], filesep);
end
if ~exist(build_dir, 'dir')
    mkdir(build_dir);
end
end
