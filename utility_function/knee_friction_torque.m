function tau_f = knee_friction_torque(qd, tau_motor, tau_s, tau_c, b_vis)
% knee_friction_torque  膝关节摩擦力矩：静摩擦 τ_s，库仑+粘性 τ_c*sign(qd)+b*qd
%
% 低速 |qd|<vel_eps 时按静摩擦（与施加力矩同向限制，幅值上限 τ_s）；
% 否则库仑 + 粘性。供正/逆动力学对比、FD 积分时扣除摩擦使用。
%
% 输入：qd 角速度，tau_motor 该关节当前力矩，tau_s/tau_c/b_vis 摩擦参数
% 输出：tau_f 摩擦力矩（标量）

vel_eps = 1e-4;
if abs(qd) < vel_eps
    tau_f = sign(tau_motor) * min(abs(tau_motor), tau_s);
    if tau_motor == 0, tau_f = 0; end
else
    tau_f = tau_c * sign(qd) + b_vis * qd;
end
end
