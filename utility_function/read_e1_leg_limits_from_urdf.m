function [joint_velocity_max, joint_lower_upper] = read_e1_leg_limits_from_urdf(urdf_file, default_velocity)
% read_e1_leg_limits_from_urdf  从 E1.urdf 读取双腿 12 关节的速度限与位置限
%
% 用法:
%   [joint_velocity_max, joint_lower_upper] = read_e1_leg_limits_from_urdf(urdf_file)
%   [joint_velocity_max, joint_lower_upper] = read_e1_leg_limits_from_urdf(urdf_file, default_velocity)
%
% 输入:
%   urdf_file        - E1.urdf 路径
%   default_velocity - 未找到 limit/velocity 时使用的默认速度 (rad/s)，默认 5.0
%
% 输出:
%   joint_velocity_max - containers.Map，关节名(用户名) -> velocity (rad/s)
%   joint_lower_upper   - struct，.lower / .upper 为 containers.Map，关节名 -> rad

if nargin < 2
    default_velocity = 5.0;
end

[all_joint_names, joint_name_mapping] = get_e1_leg_joint_names();
joint_velocity_max = containers.Map();
joint_lower_upper = struct('lower', containers.Map(), 'upper', containers.Map());

try
    xmlDoc = xmlread(urdf_file);
    jointNodes = xmlDoc.getElementsByTagName('joint');
    numJoints = jointNodes.getLength();

    for i = 1:length(all_joint_names)
        joint_name = all_joint_names{i};
        urdf_joint_name = joint_name_mapping(joint_name);
        joint_velocity_max(joint_name) = default_velocity;
        joint_lower_upper.lower(joint_name) = -pi;
        joint_lower_upper.upper(joint_name) = pi;

        for j = 0:numJoints - 1
            jointNode = jointNodes.item(j);
            if ~strcmp(char(jointNode.getAttribute('name')), urdf_joint_name)
                continue;
            end
            limitElement = jointNode.getElementsByTagName('limit').item(0);
            if isempty(limitElement)
                break;
            end
            vel = str2double(limitElement.getAttribute('velocity'));
            if ~isnan(vel) && vel > 0
                joint_velocity_max(joint_name) = vel;
            end
            low = str2double(limitElement.getAttribute('lower'));
            upp = str2double(limitElement.getAttribute('upper'));
            if ~isnan(low), joint_lower_upper.lower(joint_name) = low; end
            if ~isnan(upp), joint_lower_upper.upper(joint_name) = upp; end
            break;
        end
    end
catch ME
    warning('读取URDF失败: %s，所有关节使用默认速度 %.2f rad/s', ME.message, default_velocity);
    for i = 1:length(all_joint_names)
        joint_velocity_max(all_joint_names{i}) = default_velocity;
        joint_lower_upper.lower(all_joint_names{i}) = -pi;
        joint_lower_upper.upper(all_joint_names{i}) = pi;
    end
end
end
