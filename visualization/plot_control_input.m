function plot_control_input(velocities, params)
% plot_control_input - AUV实时速度/控制输入曲线（平滑处理）
%
% 输入:
%   velocities: steps x n 各AUV实时速度 (m/s)
%   params: 参数结构体

    figure('Name', 'AUV控制输入', 'Position', [80 80 950 600]);
    steps = size(velocities, 1);
    n = size(velocities, 2);
    t = params.sim.pre_release_time + (0:steps-1) * params.algorithm.dt;
    colors = lines(n);

    % 平滑窗口（自适应大小）
    win = min(9, max(3, floor(steps / 20)));
    if mod(win, 2) == 0, win = win + 1; end

    subplot(2,1,1);
    hold on;
    for i = 1:n
        v_smooth = smoothdata(velocities(:, i), 1, 'movmean', win);
        plot(t, v_smooth, 'Color', colors(i,:), 'LineWidth', 1.3, ...
             'DisplayName', sprintf('AUV %d', i));
    end
    yline(params.agent.max_speed, 'k--', 'LineWidth', 1.5, ...
          'DisplayName', sprintf('v_{max}=%.1f m/s', params.agent.max_speed));
    hold off;
    xlabel('泄漏时间 (s)', 'FontSize', 11);
    ylabel('线速度 v_i (m/s)', 'FontSize', 11);
    title('各AUV实时速度', 'FontSize', 13);
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    ylim([0, params.agent.max_speed * 1.15]);
    set(gca, 'FontSize', 10);

    subplot(2,1,2);
    avg_speed = smoothdata(mean(velocities, 2), 1, 'movmean', win);
    max_speed_t = smoothdata(max(velocities, [], 2), 1, 'movmean', win);
    plot(t, avg_speed, 'b-', 'LineWidth', 2, 'DisplayName', '平均速度');
    hold on;
    plot(t, max_speed_t, 'r--', 'LineWidth', 1.5, 'DisplayName', '最大速度');
    yline(params.agent.max_speed, 'k--', 'LineWidth', 1.2, 'DisplayName', 'v_{max}');
    hold off;
    xlabel('泄漏时间 (s)', 'FontSize', 11);
    ylabel('速度 (m/s)', 'FontSize', 11);
    title('集群速度统计', 'FontSize', 13);
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    ylim([0, params.agent.max_speed * 1.15]);
    set(gca, 'FontSize', 10);
end
