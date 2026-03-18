% check_e1_gravity  简单检查 E1 动力学中重力加速度是否生效、方向和量纲是否正确
%
% 方法：零位、零速、零加速度时 τ = G（纯重力项）。将重力改为 2*g 再算一次，
% 应有 G2 ≈ 2*G1；将重力改为 -g 则 G_flip ≈ -G1。据此判断重力是否设对。
%
% 用法：确保 robot_algorithm 与 utility_function 已在路径后运行
%   check_e1_gravity

clear all;
base = fileparts(mfilename('fullpath'));
root = fullfile(base, '..', '..');
addpath(genpath(fullfile(root, 'robot_algorithm')));
addpath(genpath(fullfile(root, 'utility_function')));

fprintf('========== E1 重力设置检查 ==========\n\n');

[robot, n] = get_e1_full_robot();
g_vec = robot.Gravity;

%% URDF 总质量（从文件读 <mass value="..."/>，不依赖 rigidBody 版本）
urdf_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'noetix_description', 'urdf', 'E1.urdf');
total_mass_urdf = 0;
if exist(urdf_path, 'file')
    txt = fileread(urdf_path);
    vals = regexp(txt, '<mass\s+value="([^"]+)"', 'tokens');
    for k = 1:numel(vals)
        total_mass_urdf = total_mass_urdf + str2double(vals{k}{1});
    end
end

fprintf('当前 robot.Gravity = [%.4f; %.4f; %.4f] (世界系，单位 m/s^2)\n', g_vec(1), g_vec(2), g_vec(3));
fprintf('约定：世界系 Z 向上，重力向下 = [0; 0; -9.81]。整机与单链(左/右腿、臂)均用同一世界系重力。\n');
fprintf('URDF 中所有连杆质量之和 = %.2f kg  （若实机约 100 kg 而此处远小于 100，请检查 URDF 的 <inertial><mass>）\n\n', total_mass_urdf);

%% 零位下 τ = G
q0 = zeros(n, 1);
qd0 = zeros(n, 1);
qdd0 = zeros(n, 1);
G1 = inverseDynamics(robot, q0', qd0', qdd0');
G1 = G1(:);
fprintf('零位 q=0, qd=0, qdd=0 时，重力项 G (N·m):\n');
fprintf('  ||G|| = %.4f,  max|G| = %.4f\n', norm(G1), max(abs(G1)));
% 整机顺序与 URDF 一致：左臂 1-3, 右臂 4-6, 左腿 7-12, 右腿 13-18
idx_la = 1:3; idx_ra = 4:6; idx_ll = 7:12; idx_rl = 13:18;
maxG_la = max(abs(G1(idx_la))); maxG_ra = max(abs(G1(idx_ra)));
maxG_ll = max(abs(G1(idx_ll))); maxG_rl = max(abs(G1(idx_rl)));
fprintf('  各关节 |G|: '); fprintf('%6.2f ', abs(G1)); fprintf(' N·m\n');
fprintf('  按肢体(左臂1-3 右臂4-6 左腿7-12 右腿13-18): 左臂 max|G|=%.2f, 右臂 max|G|=%.2f, 左腿 max|G|=%.2f, 右腿 max|G|=%.2f\n', maxG_la, maxG_ra, maxG_ll, maxG_rl);
if max(maxG_ll, maxG_rl) > 0.1 && abs(maxG_ll - maxG_rl) / max(maxG_ll, maxG_rl) > 0.5
    fprintf('  注意：左腿与右腿 max|G| 差异较大，对称人形零位下应接近。\n');
end
fprintf('\n');

%% 重力加倍：用临时 copy，不污染缓存
robot_2g = copy(robot);
robot_2g.Gravity = 2 * g_vec;
G2 = inverseDynamics(robot_2g, q0', qd0', qdd0');
G2 = G2(:);
err_scale = norm(G2 - 2*G1);
fprintf('将 Gravity 设为 2*g 后，零位下得到 G2。检查 G2 ≈ 2*G1：\n');
fprintf('  ||G2 - 2*G1|| = %.2e  (应接近 0)\n', err_scale);
if err_scale < 1e-6 * (norm(G1) + 1)
    fprintf('  => 重力幅值已正确参与计算。\n\n');
else
    fprintf('  => 警告：G2 与 2*G1 差异较大，请检查 Gravity 是否被正确使用。\n\n');
end

%% 重力反向：G 应反向
robot_neg_g = copy(robot);
robot_neg_g.Gravity = -g_vec;
G_flip = inverseDynamics(robot_neg_g, q0', qd0', qdd0');
G_flip = G_flip(:);
err_sign = norm(G_flip + G1);
fprintf('将 Gravity 设为 -g 后，零位下得到 G_flip。检查 G_flip ≈ -G1：\n');
fprintf('  ||G_flip + G1|| = %.2e  (应接近 0)\n', err_sign);
if err_sign < 1e-6 * (norm(G1) + 1)
    fprintf('  => 重力方向已正确参与计算。\n\n');
else
    fprintf('  => 警告：方向检查未通过。\n\n');
end

fprintf('结论：若上面两项均接近 0，则当前 Gravity 在动力学中生效正确。\n');
fprintf('若与实测/仿真对比仍不对，请确认：(1) 仿真/实测是否也是 9.81；(2) 世界系是否 Z 向上。\n\n');

%% 重力矩量级说明
fprintf('---------- 重力矩量级说明 ----------\n');
expected_scale = total_mass_urdf * 9.81 * 0.1;  % 量级: M*g*0.1m 力臂
maxG = max(abs(G1));
fprintf('总质量 %.1f kg，零位 max|G| = %.2f N·m。\n', total_mass_urdf, maxG);
fprintf('零位站直时重力沿 -Z，质心接近支点正上方、力臂小，G 可很小；屈膝/抬腿时 G 会增大。\n');
fprintf('自检：若将 Gravity 改为 [0;9.81;0]（相当于机器人“侧放”，重力沿 Y），零位下 G 应在 M*g*0.1m 量级（≈%.0f N·m），可验证惯性与重力计算正确。\n', expected_scale);
fprintf('若同一 URDF 在 Gazebo 中仿真正确，可认为惯性参数可信。\n');
fprintf('-------------------------------------\n');

%% 关于 G 的大小
fprintf('---------- 关于重力矩 G ----------\n');
fprintf('G 为各关节平衡重力所需力矩，取决于质量、质心与位形。零位站直时多由结构竖直传力，关节力矩可较小。\n');
fprintf('-----------------------------------------\n');
fprintf('\n说明：机器人在空中被固定时，世界系重力仍为 [0,0,-9.81]。单链(左/右腿、臂)计算时，当前 E1 各链树根与世界同向，故也用该重力；若链根相对世界有安装角，需在链基座系下设对应重力方向。\n');
