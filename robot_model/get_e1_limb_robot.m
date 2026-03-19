function [robot_limb, n_dof] = get_e1_limb_robot(limb, urdf_name)
% get_e1_limb_robot  由 URDF 生成指定肢体的 subtree 模型与自由度
%
% 用法:
%   [robot_limb, n_dof] = get_e1_limb_robot(limb)
%   [robot_limb, n_dof] = get_e1_limb_robot(limb, 'E1.urdf')
%
% 输入:
%   limb     - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
%   urdf_name - URDF 文件名（可选），默认为 'E1.urdf'
%               路径为默认：工程根下 noetix_description/urdf/
%
% 输出:
%   robot_limb - rigidBodyTree，DataFormat='row'，仅该肢体
%   n_dof      - 自由度：腿 6，臂 3

if nargin < 2 || isempty(urdf_name)
    urdf_name = 'E1.urdf';
end
urdf_name = char(urdf_name);

valid = {'left_leg', 'right_leg', 'left_arm', 'right_arm'};
limb = char(limb);
if ~ismember(limb, valid)
    error('get_e1_limb_robot: 未知肢体 ''%s''，可选: %s', limb, strjoin(valid, ', '));
end

% 默认路径：本仓库根目录下 noetix_description/urdf
repo_root = fileparts(fileparts(mfilename('fullpath')));
default_urdf_dir = fullfile(repo_root, 'noetix_description', 'urdf');
urdf_path = fullfile(default_urdf_dir, urdf_name);

if ~exist(urdf_path, 'file')
    % 兜底：有些工程把 URDF 放在 repo_root/urdf/ 下，或运行目录不在仓库根导致默认路径失效
    alt1 = fullfile(repo_root, 'urdf', urdf_name);
    if exist(alt1, 'file')
        urdf_path = alt1;
    else
        % 再兜底：使用 utility_function/get_e1_urdf_path 的搜索顺序（若在路径中）
        if exist('get_e1_urdf_path', 'file')
            try
                urdf_path = get_e1_urdf_path(fullfile(repo_root, 'noetix_description', 'urdf', urdf_name));
            catch
                % ignore, fallthrough to error
            end
        end
    end
end

if ~exist(urdf_path, 'file')
    error('找不到 URDF: %s', urdf_path);
end
% 缓存键含文件修改时间，URDF 保存后会自动重新加载，避免质量/惯性与文件不一致
d = dir(urdf_path);
cache_key = [urdf_path '#' num2str(d.datenum)];

persistent cache
if isempty(cache)
    cache = containers.Map();
end
if ~cache.isKey(cache_key)
    g = get_e1_gravity();
    robot_full = importrobot(urdf_path);
    robot_full.DataFormat = 'row';
    robot_full.Gravity = g;
    start_bodies = struct(...
        'left_leg',  'leg_l1_link', ...
        'right_leg', 'leg_r1_link', ...
        'left_arm',  'arm_l1_link', ...
        'right_arm', 'arm_r1_link');
    limb_robots = struct();
    limb_dofs = struct('left_leg', 6, 'right_leg', 6, 'left_arm', 3, 'right_arm', 3);
    for fn = fieldnames(start_bodies)'
        name = fn{1};
        try
            rb = subtree(robot_full, start_bodies.(name));
            rb.DataFormat = 'row';
            rb.Gravity = g;
            limb_robots.(name) = rb;
        catch ME
            error('提取肢体 %s 失败: %s', name, ME.message);
        end
    end
    cache(cache_key) = struct('limb_robots', limb_robots, 'limb_dofs', limb_dofs);
end
s = cache(cache_key);
robot_limb = s.limb_robots.(limb);
robot_limb.Gravity = get_e1_gravity();
n_dof = s.limb_dofs.(limb);
nj = robot_limb.NumBodies;
if nj ~= n_dof
    error('get_e1_limb_robot: 肢体 %s 期望 %d 关节，subtree 为 %d', limb, n_dof, nj);
end
end
