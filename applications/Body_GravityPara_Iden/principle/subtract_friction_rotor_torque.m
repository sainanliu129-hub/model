function tau_comp = subtract_friction_rotor_torque(qd_bar, qdd_bar, friction_params, I_a)
% subtract_friction_rotor_torque  计算摩擦 + 电机转子惯量产生的力矩（用于辨识前补偿）
%
% τ_meas = τ_rb + τ_friction + τ_rotor  =>  τ_rb = τ_meas - tau_comp
% 动力学参数辨识时用 τ_rb 与 Y 拟合，即辨识前先减去 tau_comp。
%
% 输入：
%   qd_bar   - M×n 角速度 (rad/s)
%   qdd_bar  - M×n 角加速度 (rad/s²)
%   friction_params - 结构体，支持两种模型：
%     (1) 库伦+粘滞：.tau_c_pos, .tau_c_neg, .b（各 1×n 或标量）
%     (2) Stribeck：.model='stribeck_viscous'，且 .tau_s, .tau_c_pos, .tau_c_neg,
%         .v_s, .b, .tau0（各 1×n）；公式 τ_f=[τ_c+(τ_s−τ_c)exp(−(|q̇|/v_s)²)]sign(q̇)+b·q̇+τ_0
%     若某字段缺失或为空，则该项不计算（视为 0）。
%   I_a      - 电机转子惯量 (kg·m²)，1×n 或标量；缺失或空则 τ_rotor = 0
%
% 输出：
%   tau_comp - M×n，τ_friction + τ_rotor，即应从实测力矩中减去的量
%
% 摩擦模型：
%   库伦+粘滞：qd>0 τ_f=tau_c_pos+b*qd，qd<0 τ_f=tau_c_neg+b*qd，qd=0 τ_f=0
%   Stribeck：τ_f=[τ_c+(τ_s−τ_c)exp(−(|q̇|/v_s)²)]sign(q̇)+b·q̇+τ_0，τ_c 按 q̇ 正负取 tau_c_pos/neg
%  转子惯量：τ_rotor(j) = I_a(j) * qdd(j)

[M, n] = size(qd_bar);
if size(qdd_bar, 1) ~= M || size(qdd_bar, 2) ~= n
    error('subtract_friction_rotor_torque: qdd_bar 须与 qd_bar 同尺寸 M×n。');
end

tau_comp = zeros(M, n);

% 摩擦力矩
if nargin >= 3 && isstruct(friction_params) && ~isempty(friction_params)
    use_stribeck = (isfield(friction_params, 'model') && ...
        strcmpi(friction_params.model, 'stribeck_viscous')) || ...
        (isfield(friction_params, 'tau_s') && isfield(friction_params, 'v_s'));
    if use_stribeck
        tau_s     = get_param_vec(friction_params, 'tau_s', n);
        tau_c_pos = get_param_vec(friction_params, 'tau_c_pos', n);
        tau_c_neg = get_param_vec(friction_params, 'tau_c_neg', n);
        v_s       = get_param_vec(friction_params, 'v_s', n);
        b         = get_param_vec(friction_params, 'b', n);
        tau0      = get_param_vec(friction_params, 'tau0', n);
        if isempty(tau_s), tau_s = zeros(1, n); end
        if isempty(tau_c_pos), tau_c_pos = zeros(1, n); end
        if isempty(tau_c_neg), tau_c_neg = zeros(1, n); end
        if isempty(v_s), v_s = 1e-6 * ones(1, n); else, v_s(v_s <= 0) = 1e-6; end
        if isempty(b), b = zeros(1, n); end
        if isempty(tau0), tau0 = zeros(1, n); end
        qd = qd_bar;      % M×n
        abs_qd = abs(qd);
        vs_safe = max(v_s, 1e-6);  % 1×n
        % τ_c 按 qd 正负取 tau_c_pos / tau_c_neg，得到 M×n
        tau_c = (qd > 0) .* tau_c_pos + (qd < 0) .* tau_c_neg;
        stribeck_br = tau_c + (tau_s - tau_c) .* exp(-(abs_qd ./ vs_safe).^2);
        tau_f = stribeck_br .* sign(qd) + b .* qd + tau0;
        tau_f(qd == 0) = 0;  % 零速取 0
        tau_comp = tau_comp + tau_f;
    else
        tau_c_pos = get_param_vec(friction_params, 'tau_c_pos', n);
        tau_c_neg = get_param_vec(friction_params, 'tau_c_neg', n);
        b         = get_param_vec(friction_params, 'b', n);
        if ~isempty(tau_c_pos) && ~isempty(tau_c_neg) && ~isempty(b)
            qd = qd_bar;
            tau_f = (qd > 0) .* (tau_c_pos + b .* qd) + ...
                    (qd < 0) .* (tau_c_neg + b .* qd);
            tau_comp = tau_comp + tau_f;
        end
    end
end

% 转子惯量力矩 τ_rotor = I_a .* qdd
if nargin >= 4 && ~isempty(I_a)
    Ia = get_param_vec_scalar_or_vec(I_a, n);
    tau_comp = tau_comp + qdd_bar .* Ia;  % (M×n) .* (1×n) 广播
end

    function v = get_param_vec(s, fld, njoints)
        if ~isfield(s, fld)
            v = [];
            return;
        end
        x = s.(fld);
        if isempty(x)
            v = [];
            return;
        end
        x = x(:).';
        if isscalar(x)
            v = repmat(x, 1, njoints);
        else
            n_avail = min(numel(x), njoints);
            v = x(1:n_avail);
            if numel(v) < njoints
                v = [v, zeros(1, njoints - numel(v))];
            end
        end
    end

    function v = get_param_vec_scalar_or_vec(x, njoints)
        x = x(:).';
        if isscalar(x)
            v = repmat(x, 1, njoints);
        else
            n_avail = min(numel(x), njoints);
            v = x(1:n_avail);
            if numel(v) < njoints
                v = [v, zeros(1, njoints - numel(v))];
            end
        end
    end
end
