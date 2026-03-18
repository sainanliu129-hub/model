function Cond_H = Objective_E1_limb( Coefficient, wf, Calculate_Num, Calculate_Interval, Calculate_Init, Sampling_Time, DOF, Num_Design_Variate_OneDof, limb )
% Objective_E1_limb  以 E1 单肢辨识矩阵条件数为目标（最小化）
%
% 与 standalone 一致：方案 B = 列归一化 + rref 取独立列 + κ(W_min)=sqrt(cond(W_min'*W_min))。
% 由 cond_value_E1_limb 计算，Inf/异常时返回 1e15。
%
% 输入:
%   Coefficient  - 1×(DOF*11) 所有关节傅里叶系数
%   limb        - 'left_leg' | 'right_leg' | 'left_arm' | 'right_arm'
% 依赖: cond_value_E1_limb（本目录）, Exciting_Trajectory（本目录）

COND_INF_PENALTY = 1e15;

try
    k = 1:Calculate_Num;
    t_sample = (Calculate_Interval.*(k-1)+Calculate_Init)*Sampling_Time;

    for Joint = 1:DOF
        Extra_Coe(Joint,1:Num_Design_Variate_OneDof) = Coefficient(1,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
    end

    [ Qn, dQn, ddQn ] = Exciting_Trajectory( Extra_Coe, t_sample, wf );

    refPos = Qn';
    refVel = dQn';
    refAcc = ddQn';

    Cond_H = cond_value_E1_limb( limb, refPos, refVel, refAcc, 1, 1 );
    if ~isfinite(Cond_H) || Cond_H <= 0
        Cond_H = COND_INF_PENALTY;
    end
catch
    Cond_H = COND_INF_PENALTY;
end
end
