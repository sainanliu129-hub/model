function urdf_path = get_e1_urdf_path(optional_urdf_path)
% get_e1_urdf_path  查找 E1.urdf（双腿+双臂）的路径
%
% 用法:
%   urdf_path = get_e1_urdf_path()
%   urdf_path = get_e1_urdf_path('path/to/E1.urdf')  % 若传入且存在则直接返回
%
% 查找顺序：传入路径(若给定) -> 工作目录 -> 当前文件向上 3/2 级 -> applications 下向上 3 级

if nargin >= 1 && ~isempty(optional_urdf_path) && exist(optional_urdf_path, 'file')
    urdf_path = optional_urdf_path;
    return;
end

current_dir = fileparts(mfilename('fullpath'));
possible_paths = {
    fullfile('noetix_description', 'urdf', 'E1.urdf');
    fullfile(current_dir, '..', 'noetix_description', 'urdf', 'E1.urdf');
    fullfile(pwd, 'noetix_description', 'urdf', 'E1.urdf');
};

for i = 1:length(possible_paths)
    if exist(possible_paths{i}, 'file')
        urdf_path = possible_paths{i};
        return;
    end
end

error('未找到 E1.urdf。请从仓库根目录运行，或将 noetix_description/urdf/E1.urdf 放在当前目录。');
end
