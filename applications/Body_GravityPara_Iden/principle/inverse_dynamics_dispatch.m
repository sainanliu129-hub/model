function tau = inverse_dynamics_dispatch(q, qd, qdd, model)
% inverse_dynamics_dispatch  逆动力学分发入口（解耦模型选择）
%
% model.type:
%   'urdf' -> inverseDynamics(robot_limb,...)
%   'min'  -> Y_min * X_hat
%   'full' -> Y * pi_vec

if ~isfield(model, 'type') || isempty(model.type)
    error('inverse_dynamics_dispatch: model.type 不能为空。');
end
type = lower(char(model.type));
limb = get_field_or(model, 'limb', 'left_leg');
para_order = get_field_or(model, 'para_order', 1);

q_row = q(:).';
qd_row = qd(:).';
qdd_row = qdd(:).';

switch type
    case 'urdf'
        [robot_limb, ~] = get_e1_limb_robot(limb);
        tau = inverseDynamics(robot_limb, q_row, qd_row, qdd_row).';

    case 'min'
        required = {'X_hat', 'index_base'};
        assert_required(model, required, 'inverse_dynamics_dispatch(min)');
        Y = ReMatrix_E1_limb_URDF(limb, q_row, qd_row, qdd_row, 1, para_order);
        Y_min = Y(:, model.index_base);
        tau = Y_min * model.X_hat(:);

    case 'full'
        required = {'pi_vec'};
        assert_required(model, required, 'inverse_dynamics_dispatch(full)');
        Y = ReMatrix_E1_limb_URDF(limb, q_row, qd_row, qdd_row, 1, para_order);
        tau = Y * model.pi_vec(:);

    otherwise
        error('inverse_dynamics_dispatch: 不支持 model.type=%s。', type);
end

tau = tau(:);
end

function v = get_field_or(s, name, default_v)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_v;
end
end

function assert_required(s, names, who_name)
for i = 1:numel(names)
    if ~isfield(s, names{i}) || isempty(s.(names{i}))
        error('%s: 缺少必要参数 %s', who_name, names{i});
    end
end
end

