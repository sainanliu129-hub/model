function effort = get_limb_effort_from_urdf(urdf_path, limb)
% get_limb_effort_from_urdf  从 URDF 读取指定肢体 6 关节的 effort 限值 (N·m)
%
% 输入：
%   urdf_path - URDF 文件路径（如 noetix_description/urdf/E1.urdf）
%   limb      - 'left_leg' | 'right_leg'
% 输出：
%   effort    - 6×1，各关节 <limit effort="..."/>；未找到或非数字时为 NaN

joint_suffix = struct('left_leg', {{'leg_l1_joint','leg_l2_joint','leg_l3_joint','leg_l4_joint','leg_l5_joint','leg_l6_joint'}}, ...
                      'right_leg', {{'leg_r1_joint','leg_r2_joint','leg_r3_joint','leg_r4_joint','leg_r5_joint','leg_r6_joint'}});
if ~isfield(joint_suffix, limb)
    error('get_limb_effort_from_urdf: limb 须为 left_leg 或 right_leg');
end
joint_names = joint_suffix.(limb);
effort = nan(6, 1);

try
    doc = xmlread(urdf_path);
catch ME
    warning('get_limb_effort_from_urdf: 读取 URDF 失败: %s', ME.message);
    return;
end

joint_elems = doc.getElementsByTagName('joint');
for idx = 1:6
    name = joint_names{idx};
    for k = 0:joint_elems.getLength-1
        joint_node = joint_elems.item(k);
        if ~strcmp(char(joint_node.getAttribute('name')), name), continue; end
        child_nodes = joint_node.getChildNodes();
        for c = 0:child_nodes.getLength-1
            node = child_nodes.item(c);
            if node.getNodeType() ~= node.ELEMENT_NODE, continue; end
            if ~strcmp(char(node.getNodeName()), 'limit'), continue; end
            effort_str = char(node.getAttribute('effort'));
            effort_val = str2double(effort_str);
            if isfinite(effort_val)
                effort(idx) = effort_val;
            end
            break;
        end
        break;
    end
end
end
