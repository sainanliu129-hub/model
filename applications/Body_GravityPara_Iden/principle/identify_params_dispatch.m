function result = identify_params_dispatch(dataset, method, opts)
% identify_params_dispatch  辨识方法分发入口（解耦算法选择）
%
% method:
%   'min_ls'   -> identify_min
%   'full_fd'  -> identify_full_params_for_fd

if nargin < 3, opts = struct(); end
if nargin < 2 || isempty(method), method = 'min_ls'; end

method = lower(char(method));
limb = get_field_or(opts, 'limb', 'left_leg');
para_order = get_field_or(opts, 'para_order', 1);

switch method
    case 'min_ls'
        id_opts = get_field_or(opts, 'identify_min_opts', struct('use_wls', false, 'para_order', para_order));
        [X_hat, index_base, metrics] = identify_min(dataset.avg_data, limb, id_opts);
        result = struct();
        result.method = method;
        result.X_hat = X_hat;
        result.index_base = index_base;
        result.metrics = metrics;

    case 'full_fd'
        required = {'pi_cad'};
        assert_required(opts, required, 'identify_params_dispatch(full_fd)');
        fd_opts = get_field_or(opts, 'identify_full_fd_opts', struct());
        [pi_fd, info] = identify_full_params_for_fd( ...
            dataset.q, dataset.qd, dataset.qdd, dataset.tau, ...
            opts.pi_cad, limb, para_order, fd_opts);
        result = struct();
        result.method = method;
        result.pi_fd = pi_fd;
        result.info = info;

    otherwise
        error('identify_params_dispatch: 不支持 method=%s。', method);
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

