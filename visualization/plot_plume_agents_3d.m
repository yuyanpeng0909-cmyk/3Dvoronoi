function plot_plume_agents_3d(positions, trajectory, plume_state, params)
% plot_plume_agents_3d - 三维羽流、智能体和轨迹可视化
%
% 输入:
%   positions: n x 3 最终智能体位置
%   trajectory: steps x n x 3 轨迹历史
%   plume_state: 羽流网格数据（来自 update_plume）
%   params: 参数结构体

    figure('Name', '自适应覆盖最终状态', 'Position', [100 100 1000 750]);

    C_max = max(plume_state.C(:));
    hold on;

    if C_max > 0
        th_outer = C_max * params.plume.boundary_threshold;
        [faces_o, verts_o] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_outer);
        if ~isempty(faces_o)
            patch('Faces', faces_o, 'Vertices', verts_o, ...
                  'FaceAlpha', 0.10, 'FaceColor', [1 0.6 0.2], 'EdgeColor', 'none', ...
                  'DisplayName', sprintf('扩散边界(%.0f%%)', params.plume.boundary_threshold * 100));
        end

        th_inner = C_max * 0.25;
        [faces_i, verts_i] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_inner);
        if ~isempty(faces_i)
            patch('Faces', faces_i, 'Vertices', verts_i, ...
                  'FaceAlpha', 0.18, 'FaceColor', [1 0.35 0], 'EdgeColor', 'none', ...
                  'DisplayName', '高浓度核心');
        end

        [~, yidx] = min(abs(plume_state.y));
        C_slice = squeeze(plume_state.C(:, yidx, :));
        X_slice = squeeze(plume_state.X(:, yidx, :));
        Z_slice = squeeze(plume_state.Z(:, yidx, :));
        C_slice_vis = C_slice;
        C_slice_vis(C_slice_vis < th_outer) = NaN;

        if any(~isnan(C_slice_vis(:)))
            surf(X_slice, zeros(size(X_slice)), Z_slice, C_slice_vis, ...
                 'FaceAlpha', 0.45, 'EdgeColor', 'none', 'FaceColor', 'interp', ...
                 'DisplayName', '纵切面浓度');
            colormap(parula);
        end
    end

    n_agents = size(trajectory, 2);
    colors = lines(n_agents);
    for i = 1:n_agents
        traj_i = squeeze(trajectory(:, i, :));
        if size(traj_i, 2) == 3
            plot3(traj_i(:,1), traj_i(:,2), traj_i(:,3), '-', ...
                  'Color', colors(i,:), 'LineWidth', 2, ...
                  'DisplayName', sprintf('AUV %d轨迹', i));
            scatter3(traj_i(1,1), traj_i(1,2), traj_i(1,3), 50, ...
                     colors(i,:), 'o', 'MarkerEdgeColor', colors(i,:), ...
                     'MarkerFaceColor', 'none', 'LineWidth', 1.5, ...
                     'HandleVisibility', 'off');
            scatter3(positions(i,1), positions(i,2), positions(i,3), 150, ...
                     colors(i,:), 's', 'filled', 'MarkerEdgeColor', 'k', ...
                     'LineWidth', 1.5, 'DisplayName', sprintf('AUV %d最终位置', i));
        end
    end

    scatter3(params.plume.source_pos(1), params.plume.source_pos(2), ...
             params.plume.source_pos(3), 300, 'r', 'p', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '溢油源');

    hold off;

    if exist('C_slice_vis', 'var') && any(~isnan(C_slice_vis(:)))
        caxis([th_outer, C_max]);
        cb = colorbar;
        cb.Label.String = '浓度 (kg/m³)';
    end

    xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12); zlabel('Z (m)', 'FontSize', 12);
    title('自适应覆盖 — 动态扩散边界追踪', 'FontSize', 14);
    axis equal; view(3); grid on;
    legend('Location', 'northeastoutside', 'FontSize', 8);
    set(gca, 'FontSize', 11);
end
