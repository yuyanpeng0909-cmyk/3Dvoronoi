function plot_monitoring_dashboard(H_adaptive, H_static, H_random, boundary_distances, plume_extent, monitored_fraction, trajectory, params)
% plot_monitoring_dashboard - 智能体监测效果综合展示
%
% 输入:
%   H_adaptive, H_static, H_random: 覆盖质量序列
%   boundary_distances: 平均边界距离
%   plume_extent: 羽流有效体积
%   monitored_fraction: 羽流边界被监测比例
%   trajectory: AUV轨迹
%   params: 参数结构体

    figure('Name', '智能体动态监测结果总览', 'Position', [50 50 1300 850]);
    t = (1:length(H_adaptive)) * params.algorithm.dt;
    colors = lines(params.agent.num);

    subplot(2,2,1);
    plot(t, H_static, 'r--', 'LineWidth', 1.8); hold on;
    plot(t, H_random, 'g:', 'LineWidth', 1.8);
    plot(t, H_adaptive, 'b-', 'LineWidth', 2.2);
    hold off;
    xlabel('时间 (s)'); ylabel('覆盖质量 H(t)');
    title('覆盖质量对比');
    legend('静态', '随机', '自适应', 'Location', 'best');
    grid on;

    subplot(2,2,2);
    yyaxis left;
    plot(t, boundary_distances, 'b-', 'LineWidth', 2);
    ylabel('平均边界距离 (m)');
    yyaxis right;
    plot(t, monitored_fraction * 100, 'm--', 'LineWidth', 2);
    ylabel('边界监测覆盖率 (%)');
    xlabel('时间 (s)');
    title('边界追踪与监测覆盖率');
    legend('边界距离', '监测覆盖率', 'Location', 'best');
    grid on;

    subplot(2,2,3);
    plot(t, plume_extent, 'Color', [0.9 0.35 0], 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('羽流有效体积 (m³)');
    title('溢油扩散范围动态扩大');
    grid on;

    subplot(2,2,4);
    hold on;
    for i = 1:params.agent.num
        traj_i = squeeze(trajectory(:, i, :));
        travel = [0; cumsum(sqrt(sum(diff(traj_i, 1, 1).^2, 2)))];
        plot(t, travel, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('AUV %d', i));
    end
    hold off;
    xlabel('时间 (s)'); ylabel('累计巡航距离 (m)');
    title('各AUV监测巡航投入');
    legend('Location', 'eastoutside', 'FontSize', 7);
    grid on;
end
