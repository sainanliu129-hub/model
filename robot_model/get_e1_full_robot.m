function [robot, n_joints, leg_idx] = get_e1_full_robot(urdf_name)
% get_e1_full_robot  由 URDF 生成整机 rigidBodyTree（与 Gazebo 一致：固定基座）
%
% 用法:
%   [robot, n_joints, leg_idx] = get_e1_full_robot()
%   [robot, n_joints, leg_idx] = get_e1_full_robot('E1.urdf')
%
% 输入:
%   urdf_name - URDF 文件名（可选），默认为 'E1.urdf'
%               路径为默认：工程根下 noetix_description/urdf/
%
% 输出:
%   robot    - rigidBodyTree，DataFormat='row'，Gravity=get_e1_gravity()
%   n_joints - 关节数（NumBodies-2）
%   leg_idx  - 腿在整机配置中的下标，与 URDF 一致：18 关节时为 7:18（左腿 7:12，右腿 13:18）

if nargin < 1 || isempty(urdf_name)
    urdf_name = 'E1.urdf';
end
urdf_name = char(urdf_name);

% 默认路径：本仓库根目录下 noetix_description/urdf
repo_root = fileparts(fileparts(mfilename('fullpath')));
default_urdf_dir = fullfile(repo_root, 'noetix_description', 'urdf');
urdf_path = fullfile(default_urdf_dir, urdf_name);

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
if cache.isKey(cache_key)
    s = cache(cache_key);
    robot = s.robot;
    n_joints = s.n_joints;
    leg_idx = leg_idx_from_n_joints(n_joints);
    robot.Gravity = get_e1_gravity();
    return;
end

robot = importrobot(urdf_path);
robot.DataFormat = 'row';
robot.Gravity = get_e1_gravity();
n_joints = robot.NumBodies - 2;
leg_idx = leg_idx_from_n_joints(n_joints);

cache(cache_key) = struct('robot', robot, 'n_joints', n_joints);
end

function idx = leg_idx_from_n_joints(n_joints)
% 与 E1.urdf 一致：整机顺序 左臂1-3 右臂4-6 左腿7-12 右腿13-18，腿下标 7:18
if n_joints == 18
    idx = 7:18;
else
    idx = 1:min(12, n_joints);
end
end
