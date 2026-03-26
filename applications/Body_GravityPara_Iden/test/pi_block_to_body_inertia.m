function [m, com_xyz, Ixx, Iyy, Izz, Iyz, Ixz, Ixy] = pi_block_to_body_inertia(p, para_order, force_psd)
if para_order == 1
    m = p(1);
    mx = p(2); my = p(3); mz = p(4);
    Ixx = p(5); Ixy = p(6); Ixz = p(7);
    Iyy = p(8); Iyz = p(9); Izz = p(10);
else
    Ixx = p(1); Ixy = p(2); Ixz = p(3);
    Iyy = p(4); Iyz = p(5); Izz = p(6);
    mx = p(7); my = p(8); mz = p(9);
    m = p(10);
end

m = max(m, 1e-12);
com_xyz = [mx; my; mz] / m;

if force_psd
    I3 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
    I3 = (I3 + I3.') / 2;
    [V, D] = eig(I3);
    d = max(diag(D), 1e-6);
    I3 = V * diag(d) * V.';
    Ixx = I3(1,1); Iyy = I3(2,2); Izz = I3(3,3);
    Ixy = I3(1,2); Ixz = I3(1,3); Iyz = I3(2,3);
end
end
