function plot_results(H_adaptive, H_static, H_random, params)
% plot_results - 覆盖质量对比曲线（含多项式趋势拟合与改进率标注）
%
% 输入:
%   H_adaptive, H_static, H_random: 1 x steps 覆盖质量数组
%   params: 参数结构体

    figure('Name', '覆盖质量对比', 'Position', [100 100 900 550]);

    t = (1:length(H_adaptive)) * params.algorithm.dt;

    % 原始曲线
    plot(t, H_static, 'r--', 'LineWidth', 2); hold on;
    plot(t, H_random, 'g:', 'LineWidth', 2);
    plot(t, H_adaptive, 'b-', 'LineWidth', 2.5);

    % 趋势拟合（自适应覆盖）
    trend_handle = [];
    if length(t) > 6
        p = polyfit(t, H_adaptive, min(5, length(t)-1));
        trend_handle = plot(t, polyval(p, t), 'b-.', 'LineWidth', 1.2, 'Color', [0.4 0.4 1]);
    end

    % 计算改进率
    H_a_start = mean(H_adaptive(1:min(3,length(H_adaptive))));
    H_a_end = mean(H_adaptive(end-min(2,length(H_adaptive)-1):end));
    H_s_end = mean(H_static(end-min(2,length(H_static)-1):end));
    improvement_vs_static = (H_s_end - H_a_end) / H_s_end * 100;
    improvement_self = (H_a_start - H_a_end) / H_a_start * 100;

    % 标注改进率
    text(t(end), H_a_end, sprintf('  \\downarrow %.1f%%', improvement_self), ...
         'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
    text(t(end), H_s_end, sprintf('  vs静态 +%.1f%%', improvement_vs_static), ...
         'Color', 'r', 'FontSize', 11, 'VerticalAlignment', 'top');

    hold off;
    xlabel('时间 (s)', 'FontSize', 12); ylabel('覆盖质量 H(t)', 'FontSize', 12);
    title('三种覆盖方法的覆盖质量对比', 'FontSize', 14);
    if ~isempty(trend_handle)
        legend('静态均匀覆盖', '随机覆盖', '自适应覆盖', '自适应趋势拟合', 'Location', 'best', 'FontSize', 11);
    else
        legend('静态均匀覆盖', '随机覆盖', '自适应覆盖', 'Location', 'best', 'FontSize', 11);
    end
    grid on;
    set(gca, 'FontSize', 11);
end
