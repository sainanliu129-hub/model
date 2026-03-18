function configs = e1_example_joint_configs()
% e1_example_joint_configs  E1 整机 18 维关节角示例（与 URDF 顺序一致：左臂1-3 右臂4-6 左腿7-12 右腿13-18）
%
% 用法:
%   configs = e1_example_joint_configs();
%   q = configs.q_zero;   % 零位
%   q = configs.q_knee_bent;  % 膝关节弯曲
%
% 返回 struct，字段名为配置名，值为 18×1 列向量 (rad)。
% 可用于 MATLAB 动力学计算，或导出到 Gazebo 对比（见 E1_GAZEBO_JOINT_CONFIGS.md）。

% 左臂 1-3, 右臂 4-6, 左腿 7-12, 右腿 13-18
q = zeros(18, 1);

%% 1. 零位（全 0）
configs.q_zero = q;

%% 2. 左腿膝关节弯曲（左腿 hip_pitch/膝 略弯，其余 0）
q2 = q;
% 左腿: leg_l1 yaw, l2 roll, l3 hip_pitch, l4 knee, l5 ankle_pitch, l6 ankle_roll
% 膝关节 leg_l4 弯曲约 0.5 rad
q2(10) = 0.5;   % 左腿 knee
q2(9)  = -0.25; % 左腿 hip_pitch 略收
configs.q_left_knee_bent = q2;

%% 3. 右腿膝关节弯曲（对称）
q3 = q;
q3(16) = 0.5;   % 右腿 knee
q3(15) = -0.25; % 右腿 hip_pitch
configs.q_right_knee_bent = q3;

%% 4. 双臂前抬（左臂 shoulder_pitch 为正，右臂对称）
q4 = q;
q4(1) = 0.4;    % 左臂 arm_l1 (shoulder_pitch)
q4(4) = 0.4;    % 右臂 arm_r1
configs.q_arms_raised = q4;

%% 5. 随机小幅度（用于测试，在限位内）
q5 = zeros(18, 1);
q5(1:3)   = (rand(3,1)-0.5)*0.3;   % 左臂
q5(4:6)   = (rand(3,1)-0.5)*0.3;   % 右臂
q5(7:12)  = (rand(6,1)-0.5)*0.2;   % 左腿
q5(13:18) = (rand(6,1)-0.5)*0.2;   % 右腿
configs.q_random_small = q5;

%% 6. 双腿微蹲（两膝同时弯）
q6 = q;
q6(9)  = -0.3;  q6(10) = 0.6;   % 左腿 hip_pitch, knee
q6(15) = -0.3;  q6(16) = 0.6;   % 右腿
configs.q_squat = q6;

end
