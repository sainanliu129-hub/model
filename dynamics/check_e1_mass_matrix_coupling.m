% check_e1_mass_matrix_coupling  检查 E1 整机质量矩阵是否按腿解耦
%
% 若 M(7:12, 13:18) 与 M(13:18, 7:12) 接近 0，则躯干固定时“分腿算”与“整机算”腿部分一致。
% 否则两腿通过躯干惯性耦合，分腿 FD 与整机 FD 结果会不同。
%
% 用法：在 MATLAB 中运行
%   run('robot_algorithm/Dynamics/E1/check_e1_mass_matrix_coupling.m')
% 或先 cd 到工程根目录后：
%   addpath(genpath('robot_algorithm')); addpath(genpath('utility_function'));
%   check_e1_mass_matrix_coupling

function check_e1_mass_matrix_coupling()
% 确保能调用 get_e1_full_robot（RobotModel）、massMatrix（E1）
base = fileparts(mfilename('fullpath'));
root = fullfile(base, '..', '..');
addpath(genpath(fullfile(root, 'robot_algorithm')));
addpath(genpath(fullfile(root, 'utility_function')));

[robot, n] = get_e1_full_robot();
if n ~= 18
    error('期望 18 关节，当前 %d', n);
end

fprintf('========== E1 质量矩阵块耦合检查 ==========\n');
fprintf('关节顺序(与 URDF 一致)：左臂 1:3，右臂 4:6，左腿 7:12，右腿 13:18\n\n');

% 取几个典型位形
configs = {
    '零位',    zeros(18, 1);
    '随机1',   (rand(18, 1) - 0.5) * 0.3;
    '随机2',   (rand(18, 1) - 0.5) * 0.5;
};
for c = 1:size(configs, 1)
    name = configs{c, 1};
    q = configs{c, 2};
    if isrow(q), q = q'; end
    M = massMatrix(robot, q');
    M_ll = M(7:12, 7:12);   % 左腿 7-12
    M_rr = M(13:18, 13:18); % 右腿 13-18
    M_lr = M(7:12, 13:18);  % 左腿-右腿耦合
    M_rl = M(13:18, 7:12);  % 右腿-左腿耦合

    n_ll = norm(M_ll, 'fro');
    n_rr = norm(M_rr, 'fro');
    n_lr = norm(M_lr, 'fro');
    n_rl = norm(M_rl, 'fro');

    fprintf('--- 位形: %s ---\n', name);
    fprintf('  ||M_ll||_F = %.4f,  ||M_rr||_F = %.4f\n', n_ll, n_rr);
    fprintf('  ||M_lr||_F = %.4f,  ||M_rl||_F = %.4f  (腿间耦合块)\n', n_lr, n_rl);
    fprintf('  耦合比 ||M_lr||/||M_ll|| = %.4f\n', n_lr / max(n_ll, 1e-10));
    if n_lr < 1e-6 && n_rl < 1e-6
        fprintf('  => 近似解耦，分腿算与整机算腿部分一致\n');
    else
        fprintf('  => 存在腿间耦合，分腿 FD 与整机 FD 结果会不同\n');
    end
    fprintf('\n');
end

fprintf('说明：若耦合比接近 0，可认为 M 按腿块对角，分肢=整机；否则需用整机 FD 与 Gazebo 一致。\n');
end
