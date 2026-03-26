function [t_one, q_ref, qd_ref, qdd_ref, t_full, Q12, report] = generate_minimal_dual_sine_excitation_trajectory(opts)
% generate_minimal_dual_sine_excitation_trajectory
% Minimal "dual-sine" excitation generator for E1 6-DOF leg.
%
% It is designed for quick spectrum/structure experiments (no optimization).
%
% Key model:
%   q_j(t) = q0_j + A1_j * sin(2*pi*f1*t + phi1_j) + A2_j * sin(2*pi*f2*t + phi2_j)
%
% Outputs are single-limb (one leg DOF=6) trajectories.
% If opts.export_csv_path is set, it will export a CSV with the SAME header format as
% write_leg_trajectory_csv_plan_format:
%   time, leg_l1_joint..leg_l6_joint, leg_r1_joint..leg_r6_joint
%
% Export (when enable_ramps=true) uses:
%   build_excitation_leg_trajectory_with_ramps + write_leg_trajectory_csv_plan_format
% to match the excitation trajectory workflow:
%   0 -> transition-in -> n_cycles -> transition-out -> 0, and other leg is a safe roll pose.

if nargin < 1 || isempty(opts), opts = struct(); end

% ---------------- Defaults ----------------
if ~isfield(opts,'type') || isempty(opts.type)
    opts.type = 'dual_sine_all'; % 'dual_sine_all' | 'hi_5_6' | 'coupled_2_4'
end
if ~isfield(opts,'freqs') || isempty(opts.freqs)
    opts.freqs = [0.5, 2.5]; % Hz
end
if ~isfield(opts,'amp_scale1') || isempty(opts.amp_scale1), opts.amp_scale1 = 0.3; end
if ~isfield(opts,'amp_scale2') || isempty(opts.amp_scale2), opts.amp_scale2 = 0.1; end
if ~isfield(opts,'phase_mode') || isempty(opts.phase_mode), opts.phase_mode = 'random'; end
if ~isfield(opts,'rng_seed') || isempty(opts.rng_seed), opts.rng_seed = 1; end

if ~isfield(opts,'sample_frequency') || isempty(opts.sample_frequency)
    opts.sample_frequency = 500; % Hz
end
if ~isfield(opts,'period') || isempty(opts.period)
    % If not provided, derive from f1. (Period is for the "one-cycle" grid we export/build.)
    opts.period = 1 / opts.freqs(1);
end
if ~isfield(opts,'limb') || isempty(opts.limb), opts.limb = 'left_leg'; end

if ~isfield(opts,'auto_scale_position') || isempty(opts.auto_scale_position)
    opts.auto_scale_position = true;
end
if ~isfield(opts,'auto_scale_margin') || isempty(opts.auto_scale_margin)
    opts.auto_scale_margin = 1e-9;
end

if ~isfield(opts,'do_plot') || isempty(opts.do_plot), opts.do_plot = false; end

% ---- Export controls (CSV with same column order) ----
if ~isfield(opts,'export_csv_path') || isempty(opts.export_csv_path)
    opts.export_csv_path = '';
end
if ~isfield(opts,'enable_ramps') || isempty(opts.enable_ramps), opts.enable_ramps = true; end
if ~isfield(opts,'traj_cycle') || isempty(opts.traj_cycle), opts.traj_cycle = 5; end
if ~isfield(opts,'transition_time') || isempty(opts.transition_time), opts.transition_time = 1.5; end
if ~isfield(opts,'export_other_leg_safe_6') || isempty(opts.export_other_leg_safe_6)
    opts.export_other_leg_safe_6 = local_default_other_leg_safe_6(opts.limb);
end

% For build_excitation_leg_trajectory_with_ramps max_velocity_ramp adaptation.
% If empty, the build will not adapt the transition durations.
if ~isfield(opts,'max_velocity_ramp') || isempty(opts.max_velocity_ramp)
    opts.max_velocity_ramp = [];
end

% ---------------- Bounds / Limits ----------------
J = 6;
q_min_default = [-0.61; -0.261; -2.09; 0; -0.872; -0.262];
q_max_default = [0.61; 0.436; 0.785; 2.44; 0.523; 0.262];
qd_max_default  = ([16.75; 20.1; 20.1; 13.18; 12.46; 12.46] * 0.9);
qdd_max_default = [200; 200; 400; 400; 200; 200];

if ~isfield(opts,'q_min') || isempty(opts.q_min), opts.q_min = q_min_default; end
if ~isfield(opts,'q_max') || isempty(opts.q_max), opts.q_max = q_max_default; end
if ~isfield(opts,'qd_max') || isempty(opts.qd_max), opts.qd_max = qd_max_default; end
if ~isfield(opts,'qdd_max') || isempty(opts.qdd_max), opts.qdd_max = qdd_max_default; end

if ~isfield(opts,'relax_vel') || isempty(opts.relax_vel), opts.relax_vel = 1.0; end
if ~isfield(opts,'relax_acc') || isempty(opts.relax_acc), opts.relax_acc = 1.0; end

q_min = opts.q_min(:);
q_max = opts.q_max(:);
qd_max = opts.qd_max(:);
qdd_max = opts.qdd_max(:);

if numel(q_min) ~= J || numel(q_max) ~= J || numel(qd_max) ~= J || numel(qdd_max) ~= J
    error('generate_minimal_dual_sine_excitation_trajectory: q_min/q_max/qd_max/qdd_max must all be 6x1.');
end

if ~isfield(opts,'q0') || isempty(opts.q0)
    q0 = (q_min + q_max)/2;
else
    q0 = opts.q0(:);
    if numel(q0) ~= J, error('opts.q0 must be 6x1'); end
end

freqs = opts.freqs(:).';
if numel(freqs) ~= 2
    error('opts.freqs must be [f1,f2] with 2 elements.');
end
f1 = freqs(1);
f2 = freqs(2);
w1 = 2*pi*f1;
w2 = 2*pi*f2;

Ts = 1 / opts.sample_frequency;
if ~isfinite(Ts) || Ts <= 0
    error('sample_frequency must be positive.');
end

period = opts.period;
if ~isfinite(period) || period <= 0
    error('opts.period must be positive.');
end

% ---------------- Time grid (one cycle) ----------------
N_one = round(period / Ts) + 1;
t_one = (0:N_one-1)' * Ts; % column

% ---------------- Joint mask based on trajectory type ----------------
mask = ones(J,1);
switch lower(strtrim(opts.type))
    case {'dual_sine_all','all'}
        mask = ones(J,1);
    case {'hi_5_6','hi56','hi_joint5_6'}
        mask = [0;0;0;0;1;1];
    case {'coupled_2_4','couple_2_4','coup24'}
        mask = [0;1;1;1;0;0];
    otherwise
        error('Unknown opts.type: %s', opts.type);
end

% ---------------- Amplitudes ----------------
q_range_full = (q_max - q_min); % 6x1, full range
A1 = opts.amp_scale1 * q_range_full;
A2 = opts.amp_scale2 * q_range_full;
A1 = A1 .* mask;
A2 = A2 .* mask;

% ---------------- Phases ----------------
if strcmpi(opts.phase_mode,'random')
    rng(opts.rng_seed);
    phi1 = 2*pi*rand(J,1);
    phi2 = 2*pi*rand(J,1);
else
    % deterministic default: all zeros
    phi1 = zeros(J,1);
    phi2 = zeros(J,1);
end

% For "coupled" type: enforce different phase offsets for joints 2..4.
if strcmpi(opts.type,'coupled_2_4') || strcmpi(opts.type,'couple_2_4') || strcmpi(opts.type,'coup24')
    phase_step = pi/4;
    base_phi1 = phi1(2);
    base_phi2 = phi2(2);
    for j = 2:4
        phi1(j) = base_phi1 + (j-2)*phase_step;
        phi2(j) = base_phi2 + (j-2)*phase_step;
    end
end

% ---------------- Synthesis ----------------
sin1 = sin(w1*t_one + phi1.');
cos1 = cos(w1*t_one + phi1.');
sin2 = sin(w2*t_one + phi2.');
cos2 = cos(w2*t_one + phi2.');

% q_ref: N_one x 6
q_ref = (q0.' + (A1.' .* sin1) + (A2.' .* sin2));
qd_ref  = (A1.' .* w1 .* cos1) + (A2.' .* w2 .* cos2);
qdd_ref = -(A1.' .* (w1^2) .* sin1) - (A2.' .* (w2^2) .* sin2);

% ---------------- Auto scale (position + vel + acc) ----------------
report = struct();
report.ok = true;
report.scale_vec = ones(J,1);
report.max_q = max(q_ref,[],1);
report.min_q = min(q_ref,[],1);
report.max_abs_qd = max(abs(qd_ref),[],1);
report.max_abs_qdd = max(abs(qdd_ref),[],1);

if opts.auto_scale_position
    center_pos = (q_max + q_min)/2;
    full_range_pos = (q_max - q_min);
    pos_valid_upper = center_pos + 0.45*full_range_pos;
    pos_valid_lower = center_pos - 0.45*full_range_pos;

    delta_q = q_ref - q0.'; % N_one x 6

    scale_vec = ones(J,1);
    for j = 1:J
        % Position-based scaling for this joint
        allowed_pos = pos_valid_upper(j) - q0(j);
        allowed_neg = q0(j) - pos_valid_lower(j);
        max_pos = max(delta_q(:,j));
        min_neg = min(delta_q(:,j)); % <=0 typically

        s_pos = inf;
        if max_pos > 0
            s_pos = allowed_pos / max_pos;
        end
        s_neg = inf;
        if min_neg < 0
            s_neg = allowed_neg / (-min_neg);
        end
        if isfinite(s_pos) && isfinite(s_neg)
            s_pos = min(s_pos, s_neg);
        else
            s_pos = min(s_pos, s_neg);
        end

        % Velocity-based scaling (qd scales linearly with amplitude)
        max_abs_qd = report.max_abs_qd(j);
        s_vel = inf;
        if max_abs_qd > 0
            s_vel = (qd_max(j) * opts.relax_vel) / max_abs_qd;
        end

        % Acc-based scaling (qdd scales linearly with amplitude)
        max_abs_qdd = report.max_abs_qdd(j);
        s_acc = inf;
        if max_abs_qdd > 0
            s_acc = (qdd_max(j) * opts.relax_acc) / max_abs_qdd;
        end

        s_j = min([s_pos, s_vel, s_acc]);
        if ~isfinite(s_j) || s_j <= 0, s_j = 1; end
        s_j = s_j * (1 - opts.auto_scale_margin);
        if s_j > 1, s_j = 1; end
        scale_vec(j) = s_j;
    end

    % Apply scaling to q/qd/qdd (q0 stays unchanged)
    q_ref = q0.' + delta_q .* scale_vec.';
    qd_ref = qd_ref .* scale_vec.';
    qdd_ref = qdd_ref .* scale_vec.';

    report.scale_vec = scale_vec;
    report.max_q = max(q_ref,[],1);
    report.min_q = min(q_ref,[],1);
    report.max_abs_qd = max(abs(qd_ref),[],1);
    report.max_abs_qdd = max(abs(qdd_ref),[],1);

    report.ok = all(report.max_q <= (pos_valid_upper(:) + 1e-9)) && ...
                 all(report.min_q >= (pos_valid_lower(:) - 1e-9)) && ...
                 all(report.max_abs_qd <= (qd_max(:) * opts.relax_vel + 1e-9)) && ...
                 all(report.max_abs_qdd <= (qdd_max(:) * opts.relax_acc + 1e-9));
end

% ---------------- Optional CSV export ----------------
t_full = [];
Q12 = [];
if ~isempty(opts.export_csv_path)
    safe6 = opts.export_other_leg_safe_6(:);
    if numel(safe6) ~= 6, error('export_other_leg_safe_6 must be 6x1'); end
    safe6 = safe6(:).';

    if opts.enable_ramps
        % Build full 12-DOF trajectory with transitions and other-leg safe pose.
        if exist('build_excitation_leg_trajectory_with_ramps','file') ~= 2
            error('Missing build_excitation_leg_trajectory_with_ramps on path. addpath(genpath(...)) first.');
        end

        other_leg_safe_6 = safe6;
        Ts_build = Ts;

        if isempty(opts.max_velocity_ramp)
            [t_full, Q12] = build_excitation_leg_trajectory_with_ramps( ...
                t_one, q_ref, opts.limb, opts.traj_cycle, opts.transition_time, Ts_build, other_leg_safe_6, qd_ref, qdd_ref);
        else
            [t_full, Q12] = build_excitation_leg_trajectory_with_ramps( ...
                t_one, q_ref, opts.limb, opts.traj_cycle, opts.transition_time, Ts_build, other_leg_safe_6, qd_ref, qdd_ref, opts.max_velocity_ramp);
        end

        if exist('write_leg_trajectory_csv_plan_format','file') ~= 2
            error('Missing write_leg_trajectory_csv_plan_format on path. addpath(genpath(...)) first.');
        end
        write_leg_trajectory_csv_plan_format(opts.export_csv_path, t_full, Q12);
    else
        % Direct "single-segment" CSV (no transitions) with same header.
        if strcmpi(opts.limb,'left_leg')
            Q12 = zeros(size(q_ref,1),12);
            Q12(:,1:6) = q_ref;
            Q12(:,7:12) = repmat(safe6,N_one,1);
            t_full = t_one;
        else
            Q12 = zeros(size(q_ref,1),12);
            Q12(:,1:6) = repmat(safe6,N_one,1);
            Q12(:,7:12) = q_ref;
            t_full = t_one;
        end

        if exist('write_leg_trajectory_csv_plan_format','file') == 2
            write_leg_trajectory_csv_plan_format(opts.export_csv_path, t_full, Q12);
        else
            local_write_plan_format_csv(opts.export_csv_path, t_full, Q12);
        end
    end

    % Optional: save single-cycle ref mat next to CSV (helps other scripts).
    try
        [csv_dir, base, ~] = fileparts(opts.export_csv_path);
        if isempty(csv_dir), csv_dir = '.'; end
        ref_mat = fullfile(csv_dir, [base '_ref.mat']);
        ref_t = t_one;
        ref_q = q_ref;
        ref_qd = qd_ref;
        save(ref_mat,'ref_t','ref_q','ref_qd');
        report.ref_mat_path = ref_mat;
    catch
    end
end

% ---------------- Plot (optional) ----------------
if opts.do_plot
    figure('Name','Minimal dual-sine - q/qd/qdd','Position',[100,100,1200,800]);
    for j=1:J
        subplot(3,1,1); hold on; grid on;
        plot(t_one,q_ref(:,j),'LineWidth',1.2,'DisplayName',sprintf('j%d',j));
        ylabel('q (rad)');
        subplot(3,1,2); hold on; grid on;
        plot(t_one,qd_ref(:,j),'LineWidth',1.0);
        ylabel('qd (rad/s)');
        subplot(3,1,3); hold on; grid on;
        plot(t_one,qdd_ref(:,j),'LineWidth',1.0);
        ylabel('qdd (rad/s^2)');
        xlabel('t (s)');
    end
end

% ---------------- Helpers ----------------
    function safe6 = local_default_other_leg_safe_6(limb_str)
        % Must match applications/Excitation_Trajectory_E1/excitation_trajectory:
        %   E1_ROLL_SAFE_RIGHT_LOWER = -0.436;
        %   E1_ROLL_SAFE_LEFT_UPPER  = 0.436;
        limb_l = lower(strtrim(limb_str));
        if contains(limb_l,'left')
            % left leg is excited => right leg roll lower
            safe6 = [0; -0.436; 0; 0; 0; 0];
        elseif contains(limb_l,'right')
            % right leg is excited => left leg roll upper
            safe6 = [0; 0.436; 0; 0; 0; 0];
        else
            error('opts.limb must contain left_leg or right_leg');
        end
    end

    function local_write_plan_format_csv(csv_path, t, Q12)
        % Local fallback writer (same header order).
        t = t(:);
        N = numel(t);
        if size(Q12,1) ~= N || size(Q12,2) ~= 12
            error('Q12 must be N x 12');
        end
        header_names = {'time', 'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint', ...
                        'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint'};
        fid = fopen(csv_path,'w');
        if fid == -1, error('Cannot create file: %s', csv_path); end
        cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
        fprintf(fid,'%s',header_names{1});
        for c=2:numel(header_names)
            fprintf(fid,',%s',header_names{c});
        end
        fprintf(fid,'\n');
        for i=1:N
            fprintf(fid,'%.5f',t(i));
            for j=1:12
                fprintf(fid,',%.6f',Q12(i,j));
            end
            fprintf(fid,'\n');
        end
    end

end

