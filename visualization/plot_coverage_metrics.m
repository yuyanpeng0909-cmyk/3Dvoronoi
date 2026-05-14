function plot_coverage_metrics(cr_proposed, cr_cvt, cr_lawnmower, ...
    rmse_proposed, rmse_cvt, rmse_lawnmower, params)
% plot_coverage_metrics - 动态覆盖率与边界追踪RMSE双面板对比图（平滑曲线）
%
% 输入:
%   cr_proposed, cr_cvt, cr_lawnmower: 1 x steps 动态覆盖率
%   rmse_proposed, rmse_cvt, rmse_lawnmower: 1 x steps 边界追踪RMSE
%   params: 参数结构体

    figure('Name', 'Dynamic Coverage Ratio and Boundary RMSE', 'Position', [80 80 950 700]);
    t = params.sim.pre_release_time + (0:length(cr_proposed)-1) * params.algorithm.dt;

    % 平滑窗口
    win = min(9, max(3, floor(length(cr_proposed) / 20)));
    if mod(win, 2) == 0, win = win + 1; end

    cr_p = smoothdata(cr_proposed * 100, 1, 'movmean', win);
    cr_c = smoothdata(cr_cvt * 100, 1, 'movmean', win);
    cr_l = smoothdata(cr_lawnmower * 100, 1, 'movmean', win);

    % 上图：动态覆盖率 CR(t)
    subplot(2,1,1);
    plot(t, cr_l, 'r--', 'LineWidth', 2); hold on;
    plot(t, cr_c, 'g:', 'LineWidth', 2);
    plot(t, cr_p, 'b-', 'LineWidth', 2.5);
    hold off;
    xlabel('Leak Time (s)', 'FontSize', 11);
    ylabel('Dynamic Coverage Ratio (%)', 'FontSize', 11);
    title('Dynamic Coverage Ratio over Time', 'FontSize', 13);
    legend('Lawnmower CPP', 'Standard CVT', 'Proposed CVT-DBT', 'Location', 'best', 'FontSize', 10);
    grid on;
    ylim([0 100]);
    set(gca, 'FontSize', 10);

    % 下图：边界追踪 RMSE
    subplot(2,1,2);
    r_p = smoothdata(rmse_proposed, 1, 'movmean', win);
    r_c = smoothdata(rmse_cvt, 1, 'movmean', win);
    r_l = smoothdata(rmse_lawnmower, 1, 'movmean', win);

    plot(t, r_l, 'r--', 'LineWidth', 2); hold on;
    plot(t, r_c, 'g:', 'LineWidth', 2);
    plot(t, r_p, 'b-', 'LineWidth', 2.5);

    % 感知半径参考线
    yline(params.agent.sense_radius, 'k-.', 'LineWidth', 1.2, ...
        'Label', sprintf('R_s=%dm', params.agent.sense_radius), 'FontSize', 9);
    hold off;
    xlabel('Leak Time (s)', 'FontSize', 11);
    ylabel('Boundary Tracking RMSE e_b (m)', 'FontSize', 11);
    title('Boundary Tracking RMSE Comparison', 'FontSize', 13);
    legend('Lawnmower CPP', 'Standard CVT', 'Proposed CVT-DBT', 'Location', 'best', 'FontSize', 10);
    grid on;
    set(gca, 'FontSize', 10);
end
