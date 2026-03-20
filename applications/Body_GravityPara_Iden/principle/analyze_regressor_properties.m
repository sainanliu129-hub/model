function [analysis_results, fig_reg] = analyze_regressor_properties(dataset, limb, para_order, t_equiv)
% analyze_regressor_properties  分析回归矩阵的条件数和回归量能量
%
% 输入:
%   dataset    : 包含 q_bar, qd_bar, qdd_bar 的数据集
%   limb       : 肢体名称，如 'left_leg'
%   para_order : 参数顺序 (例如 1)
%   t_equiv    : 等效时间向量，用于日志输出
%
% 输出:
%   analysis_results: 包含条件数、回归量能量等分析结果的结构体
%   fig_reg         : 回归量能量图的句柄

analysis_results = struct();
fig_reg = [];

q_bar = dataset.q_bar;
qd_bar = dataset.qd_bar;
qdd_bar = dataset.qdd_bar;
[Mm, nn] = size(q_bar);

fprintf('\n--- 分析回归矩阵属性 ---\n');

%% 条件数分析
try
    Y_full = [];
    for k = 1:Mm
        Y_one = ReMatrix_E1_limb_URDF(limb, q_bar(k,:), qd_bar(k,:), qdd_bar(k,:), 1, para_order);
        Y_full = [Y_full; Y_one];
    end
    col_norm = sqrt(sum(Y_full.^2, 1));
    col_norm(col_norm < 1e-12) = 1;
    W_norm = Y_full ./ (ones(size(Y_full,1), 1) * col_norm);
    r_cond = rank(W_norm);
    [~, ~, piv_cond] = qr(W_norm, 'vector');
    index_base_cond = sort(piv_cond(1:r_cond));
    Y_min_cond = Y_full(:, index_base_cond);
    
    cond_Y_full = cond(Y_full); 
    cond_Y_min = cond(Y_min_cond);
    cond_Wmin = cond(W_norm(:, index_base_cond));
    
    analysis_results.cond_Y_full = cond_Y_full;
    analysis_results.cond_Y_min = cond_Y_min;
    analysis_results.cond_Wmin = cond_Wmin;
    analysis_results.index_base_cond = index_base_cond;

    fprintf('  样本数 M = %d，时间 %.2f～%.2f s\n', Mm, t_equiv(1), t_equiv(end));
    fprintf('  cond(Y_full)     = %.4e  （全参 60 列，可能秩亏，仅作参考）\n', cond_Y_full);
    fprintf('  cond(Y_min)      = %.4e  （最小参数 %d 列）【最重要】\n', cond_Y_min, numel(index_base_cond));
    fprintf('  cond(W_min)      = %.4e  （列归一化后最小参数）\n', cond_Wmin);
catch ME
    fprintf('  条件数计算失败: %s\n', ME.message);
    analysis_results.cond_Y_full = nan;
    analysis_results.cond_Y_min = nan;
    analysis_results.cond_Wmin = nan;
    analysis_results.index_base_cond = [];
end
fprintf('-----------------------------------\n');

%% 回归量能量分析
try
    if ~exist('Y_full', 'var')
        error('Y_full 未定义（条件数计算失败时跳过）');
    end
    ncol = size(Y_full, 2);
    regressor_energy = sqrt(sum(Y_full.^2, 1));
    
    analysis_results.regressor_energy = regressor_energy;
    analysis_results.ncol = ncol;

    fprintf('  全参列数 = %d，||Y(:,i)||_2 最小值 = %.4e，最大值 = %.4e\n', ncol, min(regressor_energy), max(regressor_energy));
    idx_near_zero = find(regressor_energy < 1e-6 * max(regressor_energy));
    if ~isempty(idx_near_zero)
        fprintf('  列范数近似为 0 的列（未被激励）: %s\n', mat2str(idx_near_zero));
        analysis_results.idx_near_zero = idx_near_zero;
    else
        analysis_results.idx_near_zero = [];
    end

    fig_reg = figure('Name', 'Regressor_energy', 'Position', [160 160 1000 400]);
    bar(1:ncol, regressor_energy, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('参数列 i'); ylabel('||Y(:,i)||_2');
    title('Regressor energy：列范数 \approx 0 表示该参数未被激励');
    grid on;
    hold on;
    if ~isempty(idx_near_zero)
        plot(idx_near_zero, regressor_energy(idx_near_zero), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
        legend('||Y(:,i)||', '近似为 0（未激励）', 'Location', 'best');
    end
    hold off;
catch ME
    fprintf('  Regressor energy 计算/绘图失败: %s\n', ME.message);
    analysis_results.regressor_energy = [];
    analysis_results.ncol = 0;
    analysis_results.idx_near_zero = [];
end
fprintf('-----------------------------------\n');

end
