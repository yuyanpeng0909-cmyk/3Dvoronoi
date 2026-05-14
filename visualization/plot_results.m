function plot_results(H_proposed, H_cvt, H_lawnmower, params)
% plot_results - 覆盖质量对比曲线（含多项式趋势拟合与改进率标注）
%
% 三种方法: Proposed CVT-DBT / Standard CVT / Lawnmower CPP
%
% 输入:
%   H_proposed, H_cvt, H_lawnmower: 1 x steps 覆盖质量数组
%   params: 参数结构体

    figure('Name', '覆盖质量对比', 'Position', [100 100 900 550]);

    t = params.sim.pre_release_time + (0:length(H_proposed)-1) * params.algorithm.dt;

    % 平滑处理
    win = min(9, max(3, floor(length(H_proposed) / 20)));
    if mod(win, 2) == 0, win = win + 1; end
    H_p = smoothdata(H_proposed, 1, 'movmean', win);
    H_c = smoothdata(H_cvt, 1, 'movmean', win);
    H_l = smoothdata(H_lawnmower, 1, 'movmean', win);

    % 平滑曲线
    plot(t, H_l, 'r--', 'LineWidth', 2); hold on;
    plot(t, H_c, 'g:', 'LineWidth', 2);
    plot(t, H_p, 'b-', 'LineWidth', 2.5);

    % 趋势拟合（Proposed CVT-DBT）
    trend_handle = [];
    if length(t) > 6
        t_centered = t - t(1);
        p = polyfit(t_centered, H_p, min(5, length(t_centered)-1));
        trend_handle = plot(t, polyval(p, t_centered), 'b-.', 'LineWidth', 1.2, 'Color', [0.4 0.4 1]);
    end

    % 计算改进率
    H_p_start = mean(H_p(1:min(3,length(H_p))));
    H_p_end_val = mean(H_p(end-min(2,length(H_p)-1):end));
    H_c_end_val = mean(H_c(end-min(2,length(H_c)-1):end));
    H_l_end_val = mean(H_l(end-min(2,length(H_l)-1):end));
    improvement_vs_cvt = (H_c_end_val - H_p_end_val) / H_c_end_val * 100;
    improvement_vs_lawnmower = (H_l_end_val - H_p_end_val) / H_l_end_val * 100;
    improvement_self = (H_p_start - H_p_end_val) / H_p_start * 100;

    % 标注改进率
    text(t(end), H_p_end_val, sprintf('  \\downarrow %.1f%%', improvement_self), ...
         'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
    text(t(end), H_c_end_val, sprintf('  vs CVT +%.1f%%', improvement_vs_cvt), ...
         'Color', [0 0.5 0], 'FontSize', 11, 'VerticalAlignment', 'top');
    text(t(end), H_l_end_val, sprintf('  vs Lawn +%.1f%%', improvement_vs_lawnmower), ...
         'Color', 'r', 'FontSize', 11, 'VerticalAlignment', 'top');

    hold off;
    xlabel('泄漏时间 (s)', 'FontSize', 12); ylabel('覆盖质量 H(t)', 'FontSize', 12);
    title('三种覆盖方法的覆盖质量对比', 'FontSize', 14);
    if ~isempty(trend_handle)
        legend('Lawnmower CPP', 'Standard CVT', 'Proposed CVT-DBT', 'Proposed趋势拟合', 'Location', 'best', 'FontSize', 11);
    else
        legend('Lawnmower CPP', 'Standard CVT', 'Proposed CVT-DBT', 'Location', 'best', 'FontSize', 11);
    end
    grid on;
    set(gca, 'FontSize', 11);
end
