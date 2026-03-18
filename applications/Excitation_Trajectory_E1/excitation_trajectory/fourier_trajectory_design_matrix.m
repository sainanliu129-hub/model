function [A_pos, A_vel, A_acc, t] = fourier_trajectory_design_matrix(coeff, period, sample_frequency, dim, order)
% fourier_trajectory_design_matrix  由傅里叶系数与时间网格得到 pos/vel/acc 的设计矩阵
%
% 封装 generate_excitation_trajectory（同文件夹），仅返回设计矩阵与时间向量。
% 用于辨识：pos_vec = A_pos*coeff，vel_vec = A_vel*coeff，acc_vec = A_acc*coeff。
%
% 用法:
%   [A_pos, A_vel, A_acc, t] = fourier_trajectory_design_matrix(coeff, period, sample_frequency, dim, order)
%
% 系数顺序与 generate_excitation_trajectory 一致。

config = struct();
config.move_axis = (0:dim-1)';
config.init_joint = zeros(dim, 1);
config.upper_joint_bound = 1e10 * ones(dim, 1);
config.lower_joint_bound = -1e10 * ones(dim, 1);
config.max_velocity = 1e10 * ones(dim, 1);
config.max_acceleration = 1e10 * ones(dim, 1);
config.sample_frequency = sample_frequency;
config.order = order;
config.period = period;

[~, ~, ~, ~, ~, t, A_pos, A_vel, A_acc] = generate_excitation_trajectory(config, ...
    'coeff', coeff, 'output_design_matrix', true, 'validate', false);
end
