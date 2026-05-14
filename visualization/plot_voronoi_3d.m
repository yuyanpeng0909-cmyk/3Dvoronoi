function plot_voronoi_3d(agent_positions, t, params)
% plot_voronoi_3d - 三维Voronoi区域可视化（同色透明区域面片）
%
% 输入:
%   agent_positions: n x 3 智能体位置
%   t: 当前时间（用于密度计算）
%   params: 参数结构体

    domain = params.domain;
    plume_state = update_plume(t, params);
    C_max = max(plume_state.C(:));

    figure('Name', '三维Voronoi剖分', 'Position', [100 100 1050 780]);
    hold on;
    n = size(agent_positions, 1);
    colors = lines(n);

    if C_max > 0
        plume_threshold = C_max * params.plume.boundary_threshold;
        [faces_pl, verts_pl] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, plume_threshold);
        if ~isempty(faces_pl)
            patch('Faces', faces_pl, 'Vertices', verts_pl, ...
                  'FaceAlpha', 0.08, 'FaceColor', [1 0.45 0], 'EdgeColor', 'none', ...
                  'DisplayName', '溢油扩散边界');
        end
    end

    % 加密网格以保证稀疏区域的等值面提取
    nx = 45; ny = 36; nz = 32;
    x = linspace(domain.xmin, domain.xmax, nx);
    y = linspace(domain.ymin, domain.ymax, ny);
    z = linspace(domain.zmin, domain.zmax, nz);
    [X, Y, Z] = ndgrid(x, y, z);
    grid_points = [X(:), Y(:), Z(:)];

    C_grid = gaussian_plume_3d(grid_points(:,1), grid_points(:,2), grid_points(:,3), t, params);
    if max(C_grid) > 0
        active_mask = C_grid >= params.plume.boundary_threshold * max(C_grid);
    else
        active_mask = true(size(C_grid));
    end

    [~, nearest] = min(pdist2(grid_points, agent_positions), [], 2);
    active_volume = reshape(active_mask, size(X));

    for i = 1:n
        scatter3(agent_positions(i,1), agent_positions(i,2), agent_positions(i,3), ...
                 190, colors(i,:), 's', 'filled', 'MarkerEdgeColor', 'k', ...
                 'LineWidth', 1.5, 'DisplayName', sprintf('AUV %d', i));

        region_volume = reshape(nearest == i, size(X)) & active_volume;
        n_pts = nnz(region_volume);

        if n_pts < 4
            % 羽流交集极少：显示完整Voronoi单元（低透明度）
            region_full = reshape(nearest == i, size(X));
            if nnz(region_full) < 4, continue; end
            [faces_i, verts_i] = isosurface(X, Y, Z, smooth3(double(region_full), 'box', 3), 0.28);
            if ~isempty(faces_i)
                patch('Faces', faces_i, 'Vertices', verts_i, ...
                      'FaceAlpha', 0.10, 'FaceColor', colors(i,:), 'EdgeColor', 'none', ...
                      'DisplayName', sprintf('AUV %d覆盖区域', i));
            end
            continue;
        end

        % 根据点密度自适应调整平滑参数
        density_ratio = n_pts / nnz(nearest == i);
        if density_ratio < 0.05
            kernel_size = 3;
            iso_level = 0.15;
        elseif density_ratio < 0.15
            kernel_size = 3;
            iso_level = 0.22;
        else
            kernel_size = 3;
            iso_level = 0.28;
        end

        smoothed = smooth3(double(region_volume), 'box', kernel_size);
        [faces_i, verts_i] = isosurface(X, Y, Z, smoothed, iso_level);

        if isempty(faces_i)
            % 等值面提取失败：降级为散点渲染
            pts = grid_points(region_volume(:), :);
            if size(pts, 1) > 200
                idx = randperm(size(pts, 1), 200);
                pts = pts(idx, :);
            end
            scatter3(pts(:,1), pts(:,2), pts(:,3), 15, colors(i,:), 'filled', ...
                     'MarkerFaceAlpha', 0.25, 'DisplayName', sprintf('AUV %d覆盖区域', i));
        else
            patch('Faces', faces_i, 'Vertices', verts_i, ...
                  'FaceAlpha', 0.22, 'FaceColor', colors(i,:), 'EdgeColor', 'none', ...
                  'DisplayName', sprintf('AUV %d覆盖区域', i));
        end
    end

    scatter3(params.plume.source_pos(1), params.plume.source_pos(2), ...
             params.plume.source_pos(3), 320, 'r', 'p', 'filled', ...
             'MarkerEdgeColor', 'k', 'DisplayName', '溢油源');

    camlight('headlight'); lighting gouraud;
    hold off;
    xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12); zlabel('Z (m)', 'FontSize', 12);
    title(sprintf('三维Voronoi动态覆盖区域（透明区域面片，t=%.0fs）', t), 'FontSize', 14);
    axis equal; view(3); grid on;
    legend('Location', 'northeastoutside', 'FontSize', 8);
    set(gca, 'FontSize', 11);
end
