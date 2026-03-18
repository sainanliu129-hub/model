function Value_Cons_Viola = Constraints_Violation_E1( XI, wf, q_max, q_min, dq_max, ddq_max, DOF, Num_Design_Variate_OneDof, robot_limb, enable_collision_check, foot_x_min )
% Constraints_Violation_E1  计算轨迹相对关节限位/速度/加速度的违反量（供惩罚用）
%
% 与 AnaAp Heuristic TLBO 中 Constraints_Violation 思路一致，DOF 可配置以支持 E1 单腿(6)或单臂(3)。
% 可选：robot_limb + enable_collision_check 时在 t=0:0.01:Tf 上做自碰撞检测，每发生一次碰撞 +1 违反量。
% 可选：foot_x_min（默认 -0.4）时额外检查各连杆坐标系原点 x ≥ foot_x_min，每违反一点累加超出量。
%
% 输入: XI 为 1×(DOF*11)；q_max,q_min,dq_max,ddq_max 为 DOF×1 或 1×DOF；robot_limb、enable_collision_check、foot_x_min 可选。

Tf = 2*pi/wf;
t = 0:0.01:Tf;

q_max = q_max(:);
q_min = q_min(:);
dq_max = dq_max(:);
ddq_max = ddq_max(:);

for Joint = 1:DOF
    Coefficient_ExTra = XI(((Joint-1)*Num_Design_Variate_OneDof+1):Joint*Num_Design_Variate_OneDof);
    [q,dq,ddq] = Exciting_Trajectory( Coefficient_ExTra, t, wf );
    Qmax = max(q);
    Qmin = min(q);
    dQmax = max(abs(dq));
    ddQmax = max(abs(ddq));
    dq_initial = dq(1);
    ddq_initial = ddq(1);

    q_UM(Joint) = q_max(Joint) - Qmax;
    if q_UM(Joint) >= 0
        q_UM(Joint) = 0;
    end
    q_LM(Joint) = Qmin - q_min(Joint);
    if q_LM(Joint) >= 0
        q_LM(Joint) = 0;
    end

    dq_Vio(Joint) = dq_max(Joint) - dQmax;
    if dq_Vio(Joint) >= 0
        dq_Vio(Joint) = 0;
    end

    ddq_Vio(Joint) = ddq_max(Joint) - ddQmax;
    if ddq_Vio(Joint) >= 0
        ddq_Vio(Joint) = 0;
    end
    Q_Vio(Joint) = - q_UM(Joint) - q_LM(Joint) - dq_Vio(Joint) - ddq_Vio(Joint) + dq_initial^2 + ddq_initial^2;
end

Value_Cons_Viola = sum(Q_Vio);

% 自碰撞 + 腿部 x 约束违反量（复用同一次正运动学采样）
if nargin >= 9 && ~isempty(robot_limb)
    % 重建全关节轨迹矩阵（DOF×length(t)）
    Qn = zeros(DOF, length(t));
    for Joint = 1:DOF
        Coefficient_ExTra = XI(((Joint-1)*Num_Design_Variate_OneDof+1):Joint*Num_Design_Variate_OneDof);
        [q, ~, ~] = Exciting_Trajectory(Coefficient_ExTra, t, wf);
        Qn(Joint,:) = q;
    end
    % 自碰撞
    if nargin >= 10 && ~isempty(enable_collision_check) && enable_collision_check && exist('checkCollision', 'file')
        try
            collision_vio = 0;
            for ti = 1:size(Qn, 2)
                cfg = Qn(:, ti)';
                if checkCollision(robot_limb, cfg), collision_vio = collision_vio + 1; end
            end
            Value_Cons_Viola = Value_Cons_Viola + collision_vio;
        catch
        end
    end
    % 柱体避障：中心(cx,cy)，截面 half_dx × half_dy
    pillar = [-0.5, 0.0, 0.1, 0.1];   % [cx, cy, half_dx, half_dy]
    if nargin >= 11 && ~isempty(foot_x_min) && numel(foot_x_min) >= 4
        pillar = foot_x_min(1:4);   % 复用参数槽传入柱体配置
    end
    try
        baseName_fk   = robot_limb.BaseName;
        end_body_name = robot_limb.Bodies{robot_limb.NumBodies}.Name;
        pillar_vio = 0;
        for ti = 1:size(Qn, 2)
            cfg = Qn(:, ti)';
            T   = getTransform(robot_limb, cfg, end_body_name, baseName_fk);
            px  = T(1,4); py = T(2,4);
            ov_x = pillar(3) - abs(px - pillar(1));
            ov_y = pillar(4) - abs(py - pillar(2));
            if ov_x > 0 && ov_y > 0
                pillar_vio = pillar_vio + ov_x * ov_y;
            end
        end
        Value_Cons_Viola = Value_Cons_Viola + pillar_vio * 100;   % 放大使惩罚显著
    catch
    end
end

end
