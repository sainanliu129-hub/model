function [q, dq, ddq] = Exciting_Trajectory(Coefficient_ExTra, t, wf, order)
% Exciting_Trajectory  傅里叶级数激励轨迹（与 AnaAp.PhyConst.ExcitOptimi.DynIDen 一致）
%
% 每关节 2*L+3 个系数：a1,b1,...,aL,bL, q_ini, dq_ini, ddq_ini。L 为谐波阶数。
% 输入 Coefficient_ExTra 为 n_joint×(2*L+3)，每行一关节；t、wf 标量或同型向量。
% order 可选；不传则从列数推断：L = (size(.,2)-3)/2。
%
% 用法: [q, dq, ddq] = Exciting_Trajectory(Coefficient_ExTra, t, wf)
%       [q, dq, ddq] = Exciting_Trajectory(Coefficient_ExTra, t, wf, order)  % L=4 或 6 等

if nargin >= 4 && ~isempty(order)
    L = order;
else
    L = (size(Coefficient_ExTra, 2) - 3) / 2;   % 2*L+3 = 列数
end
n_joint = size(Coefficient_ExTra, 1);
% 确保 t 为行向量以便与 (n_joint x 1) 扩展为 (n_joint x N)
if isscalar(t)
    t = t(:)';
end
N = numel(t);
t = t(:)';

q   = zeros(n_joint, N);
dq  = zeros(n_joint, N);
ddq = zeros(n_joint, N);

for k = 1:L
    ak = Coefficient_ExTra(:, 2*k-1);
    bk = Coefficient_ExTra(:, 2*k);
    wt = k * wf * t;   % 1 x N
    q   = q   + (ak/(k*wf)).*sin(wt) - (bk/(k*wf)).*cos(wt);
    dq  = dq  + ak.*cos(wt) + bk.*sin(wt);
    ddq = ddq - ak.*(k*wf).*sin(wt) + bk.*(k*wf).*cos(wt);
end

q_ini   = Coefficient_ExTra(:, 2*L+1);
dq_ini  = Coefficient_ExTra(:, 2*L+2);
ddq_ini = Coefficient_ExTra(:, 2*L+3);

wt1 = wf * t;
q   = q   + q_ini - (dq_ini/wf).*sin(wt1) + (ddq_ini/(wf^2)).*cos(wt1);
dq  = dq  - (ddq_ini/wf).*sin(wt1) - dq_ini.*cos(wt1);
ddq = ddq - ddq_ini.*cos(wt1) + dq_ini.*(wf).*sin(wt1);

end
