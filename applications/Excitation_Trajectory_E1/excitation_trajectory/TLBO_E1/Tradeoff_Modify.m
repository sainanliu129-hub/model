function [Coefficient_ExTra] = Tradeoff_Modify( Coefficient_ExTra, wf, q_max, q_min, dq_max, ddq_max )
% Tradeoff_Modify  单关节傅里叶系数缩放与平移，使轨迹满足位置/速度/加速度限位
%
% 与 AnaAp.PhyConst.ExcitOptimi.DynIDen 一致：取缩放因子 Gama 满足位置范围、
% 速度上限、加速度上限，再平移 q_ini 使轨迹居中于 [q_min, q_max]。
%
% 输入: Coefficient_ExTra 为 1×(2*L+3)（L=谐波阶数，由列数推断）；q_max,q_min,dq_max,ddq_max 标量。

Tf = 2*pi/wf;
t = 0:0.01:Tf;

L = (numel(Coefficient_ExTra) - 3) / 2;
% dq_ini = sum(a_k), ddq_ini = sum(k*wf*b_k)（初值解析修正）
dq_ini = 0;
ddq_ini = 0;
for k = 1:L
    dq_ini  = dq_ini  + Coefficient_ExTra(2*k-1);
    ddq_ini = ddq_ini + k*wf*Coefficient_ExTra(2*k);
end
Coefficient_ExTra(1, 2*L+2) = dq_ini;
Coefficient_ExTra(1, 2*L+3) = ddq_ini;

[q, dq, ddq] = Exciting_Trajectory(Coefficient_ExTra, t, wf);

Qmax = max(q);
Qmin = min(q);
dQmax = max(abs(dq));
ddQmax = max(abs(ddq));

Gama_2 = dq_max / max(dQmax, 1e-10);
Gama_3 = ddq_max / max(ddQmax, 1e-10);
q_middle = (q_max + q_min) / 2;
Dq_limit = q_max - q_min;
Dq_current = max(Qmax - Qmin, 1e-10);
Gama_1 = Dq_limit / Dq_current;
Gama = min([Gama_1, Gama_2, Gama_3]);

Coefficient_ExTra = Coefficient_ExTra * Gama;
q_ini = Coefficient_ExTra(2*L+1);
q_center = (Qmax + Qmin) / 2 * Gama;
q_ini = q_ini - (q_center - q_middle);
Coefficient_ExTra(1, 2*L+1) = q_ini;

end
