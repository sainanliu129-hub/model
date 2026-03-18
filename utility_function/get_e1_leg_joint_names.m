function [all_joint_names, joint_name_mapping, urdf_names_ordered] = get_e1_leg_joint_names()
% get_e1_leg_joint_names  E1 双腿 12 关节名称及与 URDF 名称的映射
%
% 用法:
%   [all_joint_names, joint_name_mapping] = get_e1_leg_joint_names()
%   [~, ~, urdf_names] = get_e1_leg_joint_names()  % 仅取 CSV/绘图用 leg_l1_joint ... leg_r6_joint
%
% 输出:
%   all_joint_names     - 12x1 cell，用户/CSV 用名称（l_leg_hip_yaw_joint 等）
%   joint_name_mapping  - containers.Map，用户名 -> URDF 名（leg_l1_joint 等）
%   urdf_names_ordered  - 12x1 cell（可选），顺序为 leg_l1_joint ... leg_r6_joint，供绘图/表头等复用

all_joint_names = {
    'l_leg_hip_yaw_joint';
    'l_leg_hip_roll_joint';
    'l_leg_hip_pitch_joint';
    'l_leg_knee_joint';
    'l_leg_ankle_pitch_joint';
    'l_leg_ankle_roll_joint';
    'r_leg_hip_yaw_joint';
    'r_leg_hip_roll_joint';
    'r_leg_hip_pitch_joint';
    'r_leg_knee_joint';
    'r_leg_ankle_pitch_joint';
    'r_leg_ankle_roll_joint'
};

urdf_names = {
    'leg_l1_joint';
    'leg_l2_joint';
    'leg_l3_joint';
    'leg_l4_joint';
    'leg_l5_joint';
    'leg_l6_joint';
    'leg_r1_joint';
    'leg_r2_joint';
    'leg_r3_joint';
    'leg_r4_joint';
    'leg_r5_joint';
    'leg_r6_joint'
};

joint_name_mapping = containers.Map(all_joint_names, urdf_names);
% 第三输出：与 all_joint_names 同序的 URDF 短名，供 plan/PrincipleCalc 等复用，避免多处手写 12 个名称
if nargout >= 3
    urdf_names_ordered = urdf_names;
end
end
