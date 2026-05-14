function plot_coverage_regions_3d(agent_positions, plume_state, params)
% plot_coverage_regions_3d - 最终时刻AUV实际感知覆盖区域与溢油区域
%
% 输入:
%   agent_positions: n x 3 智能体位置
%   plume_state: 羽流网格数据（来自 update_plume）
%   params: 参数结构体

    domain = params.domain;
    n = size(agent_positions, 1);
    colors = lines(n);
    C_max = max(plume_state.C(:));

    figure('Name', 'Final Sensing Coverage Regions', 'Position', [100 100 1050 780]);
    hold on;

    if C_max > 0
        threshold = C_max * params.plume.boundary_threshold;
        plume_mask = plume_state.C >= threshold;
        plume_points = [plume_state.X(plume_mask), plume_state.Y(plume_mask), plume_state.Z(plume_mask)];

        [faces_pl, verts_pl] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, threshold);
        if ~isempty(faces_pl)
            patch('Faces', faces_pl, 'Vertices', verts_pl, ...
                  'FaceAlpha', 0.10, 'FaceColor', [1 0.45 0], 'EdgeColor', 'none', ...
                  'DisplayName', 'Oil plume boundary');
        end

        if ~isempty(plume_points)
            dist_matrix = pdist2(plume_points, agent_positions);
            [min_dist, nearest_agent] = min(dist_matrix, [], 2);
            covered_mask = min_dist <= params.agent.sense_radius;
            uncovered_points = plume_points(~covered_mask, :);

            if size(uncovered_points, 1) > 1800
                idx = randperm(size(uncovered_points, 1), 1800);
                uncovered_points = uncovered_points(idx, :);
            end
            if ~isempty(uncovered_points)
                scatter3(uncovered_points(:,1), uncovered_points(:,2), uncovered_points(:,3), ...
                         8, [0.45 0.45 0.45], 'filled', 'MarkerFaceAlpha', 0.18, ...
                         'DisplayName', 'Uncovered plume volume');
            end

            for i = 1:n
                pts = plume_points(covered_mask & nearest_agent == i, :);
                if size(pts, 1) > 1200
                    idx = randperm(size(pts, 1), 1200);
                    pts = pts(idx, :);
                end
                if ~isempty(pts)
                    scatter3(pts(:,1), pts(:,2), pts(:,3), 14, colors(i,:), 'filled', ...
                             'MarkerFaceAlpha', 0.35, 'DisplayName', sprintf('AUV %d sensed plume', i));
                end
            end
        end
    end

    for i = 1:n
        [sx, sy, sz] = sphere(18);
        surf(agent_positions(i,1) + params.agent.sense_radius * sx, ...
             agent_positions(i,2) + params.agent.sense_radius * sy, ...
             agent_positions(i,3) + params.agent.sense_radius * sz, ...
             'FaceColor', colors(i,:), 'FaceAlpha', 0.045, 'EdgeColor', 'none', ...
             'HandleVisibility', 'off');
        scatter3(agent_positions(i,1), agent_positions(i,2), agent_positions(i,3), ...
                 190, colors(i,:), 's', 'filled', 'MarkerEdgeColor', 'k', ...
                 'LineWidth', 1.5, 'DisplayName', sprintf('AUV %d', i));
    end

    scatter3(params.plume.source_pos(1), params.plume.source_pos(2), ...
             params.plume.source_pos(3), 320, 'r', 'p', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', 'Oil source');

    hold off;
    camlight('headlight'); lighting gouraud;
    xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12); zlabel('Z (m)', 'FontSize', 12);
    title(sprintf('Final AUV Sensing Coverage Regions (R_s = %.0f m)', params.agent.sense_radius), 'FontSize', 14);
    axis([domain.xmin domain.xmax domain.ymin domain.ymax domain.zmin domain.zmax]);
    daspect([1 1 1]); view(3); grid on;
    legend('Location', 'northeastoutside', 'FontSize', 7);
    set(gca, 'FontSize', 11);
end
