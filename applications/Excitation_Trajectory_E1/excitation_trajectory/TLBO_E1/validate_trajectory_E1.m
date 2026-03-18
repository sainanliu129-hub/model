function [ok, msg, details] = validate_trajectory_E1(refPos, refVel, refAcc, upper_joint, lower_joint, max_vel, max_acc, robot_limb, max_effort, relax_pos, relax_vel, relax_acc, enable_collision_check)
% validate_trajectory_E1  与 run_excitation_trajectory_standalone 校核逻辑完全一致
%
% 位置/速度/加速度/扭矩校核（扭矩限值 0.9*max_effort），最多 400 点，位置容差 1e-4 rad。
% 位置校核：完整范围（upper−lower）的 0.45，以中间点对称；不做 1.05 放宽。
% 可选：enable_collision_check=true 时做自碰撞检测（需 robot_limb，调 checkCollision）。

if nargin < 10, relax_pos = 1.0; end
if nargin < 11, relax_vel = 1.0; end
if nargin < 12, relax_acc = 1.0; end
if nargin < 13, enable_collision_check = true; end
tol_pos = 1e-4;
dim = size(refPos, 2);
upper_joint = upper_joint(:);
lower_joint = lower_joint(:);
max_vel = max_vel(:);
max_acc = max_acc(:);
% 校核位置限值：半宽 = 0.45×完整范围（与“设计半宽=0.4×完整范围”对应）
center_pos = (upper_joint + lower_joint) / 2;
full_range_pos = upper_joint - lower_joint;   % 完整范围
pos_valid_upper = center_pos + 0.45 * full_range_pos;
pos_valid_lower = center_pos - 0.45 * full_range_pos;
details = struct('sample_idx', {}, 'joint_idx', {}, 'type', {}, 'value', {}, 'limit', {}, 'excess', {});
ok = true;
msg = '';
N = size(refPos, 1);
idx_validate = round(linspace(1, N, min(400, N)));

for ii = 1:numel(idx_validate)
    i = idx_validate(ii);
    q = refPos(i,:)';
    qd = refVel(i,:)';
    qdd = refAcc(i,:)';
    for j = 1:dim
        if q(j) > pos_valid_upper(j) + tol_pos
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'pos_upper';
            details(1).value = q(j); details(1).limit = pos_valid_upper(j); details(1).excess = q(j) - pos_valid_upper(j);
            ok = false; msg = sprintf('时刻 %d 关节 %d 位置上超', i, j); return;
        end
        if q(j) < pos_valid_lower(j) - tol_pos
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'pos_lower';
            details(1).value = q(j); details(1).limit = pos_valid_lower(j); details(1).excess = pos_valid_lower(j) - q(j);
            ok = false; msg = sprintf('时刻 %d 关节 %d 位置下超', i, j); return;
        end
    end
    for j = 1:dim
        if abs(qd(j)) > max_vel(j) * relax_vel
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'vel';
            details(1).value = qd(j); details(1).limit = max_vel(j)*relax_vel; details(1).excess = abs(qd(j))-max_vel(j)*relax_vel;
            ok = false; msg = sprintf('时刻 %d 关节 %d 速度超限', i, j); return;
        end
    end
    for j = 1:dim
        if abs(qdd(j)) > max_acc(j) * relax_acc
            details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'acc';
            details(1).value = qdd(j); details(1).limit = max_acc(j)*relax_acc; details(1).excess = abs(qdd(j))-max_acc(j)*relax_acc;
            ok = false; msg = sprintf('时刻 %d 关节 %d 加速度超限', i, j); return;
        end
    end
    % 自碰撞：与 generate_excitation_trajectory 一致，cfg 为行向量
    if enable_collision_check && nargin >= 8 && ~isempty(robot_limb) && exist('checkCollision', 'file')
        try
            cfg = refPos(i,:);
            if size(cfg, 2) == 1, cfg = cfg'; end
            if checkCollision(robot_limb, cfg)
                details(1).sample_idx = i; details(1).joint_idx = 0; details(1).type = 'collision';
                details(1).value = NaN; details(1).limit = NaN; details(1).excess = 1;
                ok = false; msg = sprintf('时刻 %d 自碰撞', i); return;
            end
        catch
        end
    end
end

TORQUE_VALIDATE_RATIO = 0.9;
if nargin >= 9 && ~isempty(robot_limb) && ~isempty(max_effort)
    max_effort = max_effort(:);
    if numel(max_effort) >= dim
        max_effort = max_effort(1:dim);
        effort_limit = TORQUE_VALIDATE_RATIO * max_effort;
        for ii = 1:numel(idx_validate)
            i = idx_validate(ii);
            q = refPos(i,:); qd = refVel(i,:); qdd = refAcc(i,:);
            if iscolumn(q), q = q'; end
            if iscolumn(qd), qd = qd'; end
            if iscolumn(qdd), qdd = qdd'; end
            try
                tau = inverseDynamics(robot_limb, q, qd, qdd);
                tau = tau(:);
                for j = 1:dim
                    if abs(tau(j)) > effort_limit(j)
                        details(1).sample_idx = i; details(1).joint_idx = j; details(1).type = 'torque';
                        details(1).value = tau(j); details(1).limit = effort_limit(j); details(1).excess = abs(tau(j)) - effort_limit(j);
                        ok = false; msg = sprintf('时刻 %d 关节 %d 扭矩超限（校核限值=峰值%.0f%%）', i, j, TORQUE_VALIDATE_RATIO*100); return;
                    end
                end
            catch
            end
        end
    end
end
end
