function results = compute_regressor_cond_from_real_csv(csv_file, limb, para_order, opts)
% compute_regressor_cond_from_real_csv
% 读取实机 CSV 中的 q/qd（以及可选 qdd/acc），并计算辨识回归矩阵条件数。
%
% 设计目标：直接用“真实 q/qd/qdd”评估该激励数据是否具有数值辨识可行性。
%
% 用法示例：
%   results = compute_regressor_cond_from_real_csv( ...
%       'data/excitation/xxx.csv', 'left_leg', 1, struct('skip',10,'do_plot',false));
%       也可以加时间窗：
%       struct('skip',10,'do_plot',false,'t_start_s',2.0,'t_end_s',4.0)
%
% 关键说明：
% - CSV 读取使用 utility_function/read_leg_joint_csv.m，支持老/新两种表头格式。
% - 若 CSV 含 qdd_1_data / acc_ 列（在 read_leg_joint_csv_new 中会映射到 acc_leg_*），
%   则将 acc_leg_* 直接视为 qdd。
% - 若 CSV 不含 qdd，则用对 qd 的中心差分数值计算 qdd（会引入噪声，建议对 qd 预滤波）。
%
% 输出 results 包含：
%   cond_Y_full, cond_Y_min, cond_Wmin, index_base_cond, idx_near_zero, regressor_energy 等。

if nargin < 2 || isempty(limb), limb = 'left_leg'; end
if nargin < 3 || isempty(para_order), para_order = 1; end
if nargin < 4 || isempty(opts), opts = struct(); end

if ~isfield(opts,'skip') || isempty(opts.skip), opts.skip = 1; end
if ~isfield(opts,'do_plot') || isempty(opts.do_plot), opts.do_plot = true; end
if ~isfield(opts,'qd_lowpass_fc_Hz') || isempty(opts.qd_lowpass_fc_Hz), opts.qd_lowpass_fc_Hz = 0; end
if ~isfield(opts,'do_vel_acc_compare') || isempty(opts.do_vel_acc_compare)
    opts.do_vel_acc_compare = false;
end
if ~isfield(opts,'t_start_s') || isempty(opts.t_start_s)
    opts.t_start_s = [];
end
if ~isfield(opts,'t_end_s') || isempty(opts.t_end_s)
    opts.t_end_s = [];
end
if ~isfield(opts,'ref_mat_path') || isempty(opts.ref_mat_path)
    opts.ref_mat_path = '';
end

if isstring(csv_file), csv_file = char(csv_file); end

% 路径设置：找到 read_leg_joint_csv 与 analyze_regressor_properties
test_dir = fileparts(mfilename('fullpath')); %#ok<NASGU>
app_root = fullfile(fileparts(test_dir), ''); %#ok<NASGU>
repo_root = fullfile(test_dir, '..', '..', '..'); %#ok<NASGU>

% 更稳妥：显式加路径到本 repo 的相对位置
body_dir = fileparts(fileparts(mfilename('fullpath'))); % applications/Body_GravityPara_Iden
repo_root = fileparts(fileparts(body_dir));             % model_e1

addpath(fullfile(repo_root, 'applications', 'Body_GravityPara_Iden', 'principle'));
addpath(fullfile(repo_root, 'utility_function'));
addpath(fullfile(repo_root, 'applications', 'Body_GravityPara_Iden'));
if exist('ensure_body_gravity_para_iden_path','file') == 2
    ensure_body_gravity_para_iden_path();
end

if ~exist(csv_file,'file')
    % 允许 csv_file 为相对路径（相对于 Body_GravityPara_Iden/app_root）
    if ~isfolder(fullfile(body_dir,'data')) && exist(csv_file,'file') ~= 2
        error('CSV 文件不存在: %s', csv_file);
    end
    if exist(fullfile(body_dir, csv_file), 'file') == 2
        csv_file = fullfile(body_dir, csv_file);
    else
        error('CSV 文件不存在: %s', csv_file);
    end
end

data = read_leg_joint_csv(csv_file);
t = data.time(:);

if strcmpi(limb,'left_leg')
    q  = data.pos_leg_l;
    qd = data.vel_leg_l;
    if isfield(data,'acc_leg_l') && ~isempty(data.acc_leg_l)
        qdd = data.acc_leg_l;
    else
        qdd = [];
    end
elseif strcmpi(limb,'right_leg')
    q  = data.pos_leg_r;
    qd = data.vel_leg_r;
    if isfield(data,'acc_leg_r') && ~isempty(data.acc_leg_r)
        qdd = data.acc_leg_r;
    else
        qdd = [];
    end
else
    error('limb 必须是 left_leg 或 right_leg');
end

q = q(:,1:min(6,size(q,2)));
qd = qd(:,1:min(6,size(qd,2)));

% -------- Time window selection (for cond computation) --------
mask_win = true(size(t)); %#ok<NASGU>
if ~isempty(opts.t_start_s)
    mask_win = mask_win & (t >= opts.t_start_s);
end
if ~isempty(opts.t_end_s)
    mask_win = mask_win & (t <= opts.t_end_s);
end
if ~any(mask_win)
    error('compute_regressor_cond_from_real_csv: time window is empty. t_start_s=%.6g, t_end_s=%.6g', opts.t_start_s, opts.t_end_s);
end
t_start_used = t(find(mask_win,1,'first'));
t_end_used   = t(find(mask_win,1,'last'));
t  = t(mask_win);
q  = q(mask_win,:);
qd = qd(mask_win,:);
if ~isempty(qdd)
    qdd = qdd(mask_win,:);
end

if isempty(qdd)
    % 从 qd 计算 qdd（中心差分）
    dt = median(diff(t));
    if ~isfinite(dt) || dt <= 0
        dt = 0.002;
    end
    if opts.qd_lowpass_fc_Hz > 0
        % 对 qd 进行低通，避免 qdd 差分爆炸
        fs = 1/dt;
        fc = opts.qd_lowpass_fc_Hz;
        if fs > 2*fc
            [b,a] = butter(2, fc/(fs/2));
            for j = 1:6
                qd(:,j) = filtfilt(b,a,qd(:,j));
            end
        end
    end
    qdd = zeros(size(qd));
    if size(qd,1) >= 3
        qdd(2:end-1,:) = (qd(3:end,:) - qd(1:end-2,:)) / (2*dt);
        qdd(1,:)       = (qd(2,:) - qd(1,:)) / dt;
        qdd(end,:)     = (qd(end,:) - qd(end-1,:)) / dt;
    else
        qdd(:) = 0;
    end
end

% 下采样（加速 ReMatrix 计算）
skip = max(1, round(opts.skip));
idx = 1:skip:numel(t);
t_equiv = t(idx);
q_bar = q(idx,:);
qd_bar = qd(idx,:);
qdd_bar = qdd(idx,:);

dataset = struct();
dataset.q_bar = q_bar;
dataset.qd_bar = qd_bar;
dataset.qdd_bar = qdd_bar;

% ---------------- Expected vs Actual qd/qdd plot ----------------
analysis_results_expected = [];
if opts.do_vel_acc_compare
    try
        [csv_dir, base, ~] = fileparts(csv_file);
        if isempty(csv_dir), csv_dir = '.'; end

        % Locate reference single-cycle mat (best-effort)
        ref_mat = '';
        if ~isempty(opts.ref_mat_path) && exist(opts.ref_mat_path,'file') == 2
            ref_mat = opts.ref_mat_path;
        else
            cand1 = fullfile(csv_dir, [base '_ref.mat']);
            if exist(cand1,'file') == 2
                ref_mat = cand1;
            else
                % fallback: latest *_ref.mat in same folder
                fref = dir(fullfile(csv_dir,'*_ref.mat'));
                if ~isempty(fref)
                    [~,k] = sort([fref.datenum],'descend');
                    ref_mat = fullfile(csv_dir, fref(k(1)).name);
                end
            end
        end

        if isempty(ref_mat)
            fprintf('[compute_regressor_cond_from_real_csv] 未找到 *_ref.mat，跳过期望/实际速度加速度对比绘图。\n');
        else
            S = load(ref_mat);
            if isfield(S,'ref_t')
                ref_t = S.ref_t(:);
            else
                error('ref_mat 缺少 ref_t');
            end
            if isfield(S,'ref_qd')
                ref_qd = S.ref_qd;
            else
                error('ref_mat 缺少 ref_qd');
            end
            % ref_qd 通常是 CN×dim
            ref_qd = ref_qd(:, 1:min(6,size(ref_qd,2)));
            dim_ref = size(ref_qd,2);
            if dim_ref < 6
                % 如果参考只有部分关节，就只绘制对应维度（仍用前 dim_ref）
                fprintf('[compute_regressor_cond_from_real_csv] ref_qd 维度不足：dim_ref=%d，绘图将只覆盖前 %d 轴。\n', dim_ref, dim_ref);
            end

            period = ref_t(end) - ref_t(1);
            if ~isfinite(period) || period <= 0
                period = ref_t(end);
            end
            if ~isfinite(period) || period <= 0
                error('ref_t 计算 period 失败');
            end

            % Map actual time -> expected phase (modulo period)
            tau = mod(t_equiv - ref_t(1), period);

            qd_expected = zeros(size(qd_bar,1), 6);
            for j = 1:6
                if j <= dim_ref
                    qd_expected(:,j) = interp1(ref_t, ref_qd(:,j), tau, 'linear', 'extrap');
                end
            end

            % qdd expected: if ref_qdd exists use it; otherwise compute from ref_qd
            qdd_expected = [];
            if isfield(S,'ref_qdd') && ~isempty(S.ref_qdd)
                ref_qdd = S.ref_qdd;
                ref_qdd = ref_qdd(:, 1:min(6,size(ref_qdd,2)));
                qdd_expected = zeros(size(qdd_bar,1), 6);
                dim_ref2 = size(ref_qdd,2);
                for j = 1:6
                    if j <= dim_ref2
                        qdd_expected(:,j) = interp1(ref_t, ref_qdd(:,j), tau, 'linear', 'extrap');
                    end
                end
            else
                % derive qdd_expected from ref_qd
                dt_ref = median(diff(ref_t));
                if ~isfinite(dt_ref) || dt_ref <= 0, dt_ref = 1e-3; end
                qdd_ref_derived = zeros(size(ref_qd));
                for j = 1:dim_ref
                    qdd_ref_derived(:,j) = gradient(ref_qd(:,j), ref_t);
                end
                qdd_expected = zeros(size(qdd_bar,1), 6);
                for j = 1:6
                    if j <= dim_ref
                        qdd_expected(:,j) = interp1(ref_t, qdd_ref_derived(:,j), tau, 'linear', 'extrap');
                    end
                end
            end

            % Plot qd compare
            fig1 = figure('Name','Expected vs Actual qd','Position',[100,100,1200,800]);
            for j = 1:6
                subplot(3,2,j);
                plot(t_equiv, qd_expected(:,j), 'r-', 'LineWidth',1.2); hold on; grid on;
                plot(t_equiv, qd_bar(:,j), 'b--', 'LineWidth',1.0);
                ylabel(sprintf('q%d (rad/s)', j));
                if j==1, title('qd: expected (ref) vs actual (from csv)'); legend({'expected','actual'},'Location','best'); end
            end
            xlabel('t (s)');

            % Plot qdd compare
            fig2 = figure('Name','Expected vs Actual qdd','Position',[100,100,1200,800]);
            for j = 1:6
                subplot(3,2,j);
                plot(t_equiv, qdd_expected(:,j), 'r-', 'LineWidth',1.2); hold on; grid on;
                plot(t_equiv, qdd_bar(:,j), 'b--', 'LineWidth',1.0);
                ylabel(sprintf('q%d (rad/s^2)', j));
                if j==1, title('qdd: expected (derived) vs actual'); legend({'expected','actual'},'Location','best'); end
            end
            xlabel('t (s)');
        end
    catch ME
        fprintf('[compute_regressor_cond_from_real_csv] 期望/实际对比绘图失败：%s\n', ME.message);
    end
end

[analysis_results, fig_reg] = analyze_regressor_properties(dataset, limb, para_order, t_equiv);

if ~opts.do_plot && ~isempty(fig_reg) && ishandle(fig_reg)
    close(fig_reg);
end

% -------- Expected trajectory condition numbers (from CSV expected) --------
% 你反馈“期望轨迹也是.csv中的期望轨迹”，因此这里优先使用 CSV 里的 cmd_leg_* 作为 ref_q，
% 然后通过差分/梯度得到 qd/qdd，再计算 cond。
try
    if strcmpi(limb,'left_leg')
        q_ref_all = data.cmd_leg_l;
    else
        q_ref_all = data.cmd_leg_r;
    end

    if ~isempty(q_ref_all)
        q_ref_all = q_ref_all(mask_win,:);
    end

    if ~isempty(q_ref_all) && size(q_ref_all,1) == numel(t)
        q_ref_all = q_ref_all(:, 1:min(6,size(q_ref_all,2)));

        dt_ref = median(diff(t));
        if ~isfinite(dt_ref) || dt_ref <= 0, dt_ref = 1e-3; end

        % qd from q_ref (first derivative)
        qd_ref_all = zeros(size(q_ref_all));
        if size(q_ref_all,1) >= 3
            qd_ref_all(2:end-1,:) = (q_ref_all(3:end,:) - q_ref_all(1:end-2,:)) / (2*dt_ref);
            qd_ref_all(1,:)       = (q_ref_all(2,:) - q_ref_all(1,:)) / dt_ref;
            qd_ref_all(end,:)     = (q_ref_all(end,:) - q_ref_all(end-1,:)) / dt_ref;
        end

        % Optional low-pass filter on qd before differentiating to qdd
        if opts.qd_lowpass_fc_Hz > 0
            fs = 1/dt_ref;
            fc = opts.qd_lowpass_fc_Hz;
            if fs > 2*fc
                [b,a] = butter(2, fc/(fs/2));
                for j = 1:6
                    qd_ref_all(:,j) = filtfilt(b,a,qd_ref_all(:,j));
                end
            end
        end

        % qdd from qd_ref (second derivative)
        qdd_ref_all = zeros(size(qd_ref_all));
        if size(qd_ref_all,1) >= 3
            qdd_ref_all(2:end-1,:) = (qd_ref_all(3:end,:) - qd_ref_all(1:end-2,:)) / (2*dt_ref);
            qdd_ref_all(1,:)       = (qd_ref_all(2,:) - qd_ref_all(1,:)) / dt_ref;
            qdd_ref_all(end,:)     = (qd_ref_all(end,:) - qd_ref_all(end-1,:)) / dt_ref;
        end

        dataset_exp_from_csv = struct();
        dataset_exp_from_csv.q_bar  = q_ref_all(idx,:);
        dataset_exp_from_csv.qd_bar = qd_ref_all(idx,:);
        dataset_exp_from_csv.qdd_bar = qdd_ref_all(idx,:);

        [analysis_results_expected_from_csv, fig_reg2] = analyze_regressor_properties( ...
            dataset_exp_from_csv, limb, para_order, t_equiv);

        if ~opts.do_plot && ~isempty(fig_reg2) && ishandle(fig_reg2)
            close(fig_reg2);
        end

        analysis_results_expected = analysis_results_expected_from_csv;
    else
        fprintf('[compute_regressor_cond_from_real_csv] CSV 里没有有效 cmd_leg_%s，expected cond 跳过。\n', limb);
    end
catch MEexp
    fprintf('[compute_regressor_cond_from_real_csv] 期望轨迹(cond)从 CSV cmd_leg 计算失败：%s\n', MEexp.message);
end

results = analysis_results;
% 如果已经算过期望轨迹条件数，把结果也带回去
if ~isempty(analysis_results_expected)
    results.expected = analysis_results_expected;
end
results.csv_file = csv_file;
results.limb = limb;
results.para_order = para_order;
results.skip = skip;
results.N_used = numel(idx);

fprintf('\n==== Regressor condition numbers (from real csv) ====\n');
fprintf('CSV: %s\n', csv_file);
fprintf('Limb: %s, para_order=%d, N_used=%d, skip=%d\n', limb, para_order, results.N_used, skip);
fprintf('Time window: [%.3f, %.3f] s\n', t_start_used, t_end_used);
fprintf('cond(Y_full) = %.4e\n', results.cond_Y_full);
fprintf('cond(Y_min)  = %.4e (index_base_cond length=%d)\n', results.cond_Y_min, numel(results.index_base_cond));
fprintf('cond(W_min)  = %.4e\n', results.cond_Wmin);
if isfield(results,'expected') && ~isempty(results.expected)
    fprintf('\n---- Expected trajectory cond (from CSV cmd_leg) ----\n');
    fprintf('cond(Y_full)_exp = %.4e\n', results.expected.cond_Y_full);
    fprintf('cond(Y_min)_exp  = %.4e (index_base_cond length=%d)\n', results.expected.cond_Y_min, numel(results.expected.index_base_cond));
    fprintf('cond(W_min)_exp  = %.4e\n', results.expected.cond_Wmin);
end
if isfield(results,'idx_near_zero') && ~isempty(results.idx_near_zero)
    fprintf('Near-zero regressor columns (count=%d): %s\n', numel(results.idx_near_zero), mat2str(results.idx_near_zero));
end
fprintf('=========================================================\n');

end

