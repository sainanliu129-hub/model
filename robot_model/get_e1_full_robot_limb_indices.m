function [idx_left_leg, idx_right_leg, idx_left_arm, idx_right_arm] = get_e1_full_robot_limb_indices()
% get_e1_full_robot_limb_indices  检测整机配置向量中四条单链对应的索引
%
% E1.urdf 整机顺序为 左臂1-3、右臂4-6、左腿7-12、右腿13-18。本函数在零位下用质量矩阵块匹配
% 得到各肢体在整机中的实际索引（用于与单链 M/C/G 对比等）。
%
% 用法:
%   [idx_ll, idx_rl, idx_la, idx_ra] = get_e1_full_robot_limb_indices()
%
% 输出:
%   idx_left_leg  - 左腿在整机 q/M/G 中的下标，1×6
%   idx_right_leg - 右腿下标，1×6
%   idx_left_arm  - 左臂下标，1×3
%   idx_right_arm - 右臂下标，1×3

[robot_full, n] = get_e1_full_robot();
if n ~= 18
    error('get_e1_full_robot_limb_indices: 期望 18 关节，当前 %d', n);
end
q_full = zeros(n, 1);
M_full = massMatrix(robot_full, q_full');

limbs_6 = {'left_leg', 'right_leg'};
limbs_3 = {'left_arm', 'right_arm'};
idx_left_leg  = [];
idx_right_leg = [];
idx_left_arm  = [];
idx_right_arm = [];

% 6×6 块：找与 left_leg、right_leg 单链 M 最接近的整机块
for limb_name = limbs_6
    rb = get_e1_limb_robot(limb_name{1});
    q_leg = zeros(6, 1);
    M_limb = massMatrix(rb, q_leg');
    best_err = inf;
    best_idx = [];
    for start = 1 : n - 5
        idx = start : start + 5;
        err = norm(M_full(idx, idx) - M_limb, 'fro');
        if err < best_err
            best_err = err;
            best_idx = idx;
        end
    end
    if strcmp(limb_name{1}, 'left_leg')
        idx_left_leg = best_idx;
    else
        idx_right_leg = best_idx;
    end
end

% 3×3 块：找与 left_arm、right_arm 单链 M 最接近的整机块
for limb_name = limbs_3
    rb = get_e1_limb_robot(limb_name{1});
    q_arm = zeros(3, 1);
    M_limb = massMatrix(rb, q_arm');
    best_err = inf;
    best_idx = [];
    for start = 1 : n - 2
        idx = start : start + 2;
        err = norm(M_full(idx, idx) - M_limb, 'fro');
        if err < best_err
            best_err = err;
            best_idx = idx;
        end
    end
    if strcmp(limb_name{1}, 'left_arm')
        idx_left_arm = best_idx;
    else
        idx_right_arm = best_idx;
    end
end
end
