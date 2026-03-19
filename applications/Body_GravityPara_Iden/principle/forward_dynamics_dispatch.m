function qdd = forward_dynamics_dispatch(q, qd, tau, model, opts)
% forward_dynamics_dispatch  正动力学分发入口（解耦模型选择）
%
% model.type:
%   'urdf' -> forwardDynamics(robot_limb,...)
%   'min'  -> forward_dynamics_min
%   'full' -> forward_dynamics_full

if nargin < 5, opts = struct(); end
if ~isfield(model, 'type') || isempty(model.type)
    error('forward_dynamics_dispatch: model.type 不能为空。');
end
type = lower(char(model.type));

switch type
    case 'urdf'
        limb = get_field_or(model, 'limb', 'left_leg');
        [robot_limb, ~] = get_e1_limb_robot(limb);
        qdd = forwardDynamics(robot_limb, q(:).', qd(:).', tau(:).').';

    case 'min'
        required = {'X_hat', 'index_base'};
        assert_required(model, required, 'forward_dynamics_dispatch(min)');
        limb = get_field_or(model, 'limb', 'left_leg');
        para_order = get_field_or(model, 'para_order', 1);
        qdd = forward_dynamics_min(q, qd, tau, model.X_hat, model.index_base, limb, para_order);

    case 'full'
        required = {'pi_vec'};
        assert_required(model, required, 'forward_dynamics_dispatch(full)');
        limb = get_field_or(model, 'limb', 'left_leg');
        para_order = get_field_or(model, 'para_order', 1);
        qdd = forward_dynamics_full(q, qd, tau, model.pi_vec, limb, para_order, opts);

    otherwise
        error('forward_dynamics_dispatch: 不支持 model.type=%s。', type);
end
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

