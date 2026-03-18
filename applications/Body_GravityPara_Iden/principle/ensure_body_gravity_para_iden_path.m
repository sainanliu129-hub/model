function ensure_body_gravity_para_iden_path()
% ensure_body_gravity_para_iden_path  确保 Body_GravityPara_Iden 脚本所需路径已加载
%
% 本仓库 (model_e1)：将 utility_function、dynamics、robot_model 加入路径，
% 供 compare_forward_dynamics_vs_measured、compare_torque_sim_vs_id、
% compare_single_joint_torque_aligned 等脚本使用。
%
% 用法：在脚本开头调用 ensure_body_gravity_para_iden_path();

base = fileparts(mfilename('fullpath'));
% 本文件在 Body_GravityPara_Iden/principle 下，上三级为仓库根
root = fullfile(base, '..', '..', '..');
addpath(fullfile(root, 'utility_function'));
addpath(fullfile(root, 'dynamics'));
addpath(fullfile(root, 'robot_model'));
end
