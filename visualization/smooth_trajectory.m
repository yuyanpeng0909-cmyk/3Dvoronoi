function smooth_traj = smooth_trajectory(trajectory)
% smooth_trajectory - 平滑AUV轨迹用于可视化显示

    smooth_traj = trajectory;
    steps = size(trajectory, 1);
    if steps < 5
        return;
    end

    window = min(9, 2 * floor((steps - 1) / 2) + 1);
    for i = 1:size(trajectory, 2)
        for d = 1:3
            series = trajectory(:, i, d);
            smooth_traj(:, i, d) = smoothdata(series, 'movmean', window);
            smooth_traj(1, i, d) = series(1);
            smooth_traj(end, i, d) = series(end);
        end
    end
end
