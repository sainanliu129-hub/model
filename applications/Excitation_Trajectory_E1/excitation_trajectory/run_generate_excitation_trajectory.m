function run_generate_excitation_trajectory(varargin)
% run_generate_excitation_trajectory  运行傅里叶激励轨迹生成（BODY_JNT_CURRENT 辨识用）
%
% 本脚本位于 applications/Excitation_Trajectory_E1/excitation_trajectory 内；
% 需将 Body_GravityPara_Iden 加入路径以调用 ensure_body_gravity_para_iden_path。
% 用法：
%   run_generate_excitation_trajectory
%   run_generate_excitation_trajectory('optimize', true)
%   run_generate_excitation_trajectory('save_mat', 'excite_traj.mat')

% 上级目录 = Excitation_Trajectory_E1；需显式加入 Body_GravityPara_Iden 以使用 ensure_body_gravity_para_iden_path
this_dir   = fileparts(mfilename('fullpath'));
parent_dir = fullfile(this_dir, '..');                 % Excitation_Trajectory_E1
repo_root  = fullfile(parent_dir, '..', '..');         % model_e1 仓库根
addpath(parent_dir);
addpath(fullfile(repo_root, 'applications', 'Body_GravityPara_Iden'));
ensure_body_gravity_para_iden_path();                  % utility_function, dynamics, robot_model
addpath(this_dir);

%% 1. 配置（与文档 TOML 示例一致）
config = struct();
config.type = 10;                          % BODY_JNT_CURRENT
config.move_axis = [0, 1, 2, 3, 4, 5];     % 全轴 6
config.init_joint = [0; 0; 0; 0; 0; 0];    % rad
config.traj_cycle = 2;
config.sample_frequency = 200;
config.upper_joint_bound = [2.0; 1.57; 2.0; 2.0; 2.0; 2.0];
config.lower_joint_bound = [-2.0; -1.57; -2.0; -2.0; -2.0; -2.0];
config.upper_cartesian_bound = [2.0; 2.0; 2.0];
config.lower_cartesian_bound = [-2.0; -2.0; 0.0];
config.max_velocity = [3.0; 3.0; 3.0; 3.0; 3.0; 3.0];
config.max_acceleration = [7.0; 7.0; 7.0; 7.0; 7.0; 7.0];
config.order = 5;
config.period = 10;
config.sample_number = 15;

%% 2. 可选参数
p = inputParser;
addParameter(p, 'optimize', false, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'save_mat', '', @ischar);
addParameter(p, 'plot_traj', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
parse(p, varargin{:});
opts = p.Results;

%% 3. 生成轨迹
fprintf('===== 傅里叶激励轨迹生成 =====\n');
if opts.optimize
    fprintf('正在优化系数（最小化激励量代理 / 条件数）...\n');
end
[trajectory, trajPara, refPos, refVel, refAcc, t_period] = generate_excitation_trajectory(config, ...
    'optimize', opts.optimize, 'validate', true);

CN = size(refPos, 1);
dim = size(refPos, 2);
fprintf('单周期点数 CN = %d, dim = %d\n', CN, dim);
fprintf('系数个数 = %d\n', numel(trajPara));

%% 4. 绘图（单周期 refPos, refVel, refAcc）
if opts.plot_traj
    figure('Name', '激励轨迹 - 单周期 q / qd / qdd', 'Position', [50, 50, 1200, 800]);
    for j = 1:dim
        subplot(dim, 3, (j-1)*3 + 1);
        plot(t_period, refPos(:,j), 'b-'); grid on; ylabel('q (rad)');
        if j == 1, title('位置'); end
        subplot(dim, 3, (j-1)*3 + 2);
        plot(t_period, refVel(:,j), 'r-'); grid on; ylabel('qd (rad/s)');
        if j == 1, title('速度'); end
        subplot(dim, 3, (j-1)*3 + 3);
        plot(t_period, refAcc(:,j), 'k-'); grid on; ylabel('qdd (rad/s^2)');
        if j == 1, title('加速度'); end
    end
    xlabel('time (s)');
end

%% 5. 保存
if ~isempty(opts.save_mat)
    save(opts.save_mat, 'config', 'trajectory', 'trajPara', 'refPos', 'refVel', 'refAcc', 't_period');
    fprintf('已保存: %s\n', opts.save_mat);
end

fprintf('\n可填入 [excite_traj_para] traj_parameter 的系数（前 12 个）:\n');
fprintf('  %.6f ', trajPara(1:min(12,end)));
fprintf('...\n');

fprintf('===== 完成 =====\n');
end
