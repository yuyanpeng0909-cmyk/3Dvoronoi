function create_dynamic_plume_gif(trajectory, params, save_path)
% create_dynamic_plume_gif - 生成包含溢油源扩散和AUV动态监测过程的GIF
%
% 输入:
%   trajectory: steps x n x 3 AUV轨迹历史
%   params: 参数结构体
%   save_path: GIF保存路径

    steps = size(trajectory, 1);
    n_agents = size(trajectory, 2);
    colors = lines(n_agents);
    frame_count = 36;
    frame_steps = unique(round(linspace(1, steps, frame_count)));
    domain = params.domain;

    fig = figure('Name', '动态溢油扩散与AUV监测GIF', ...
                 'Position', [100 100 1180 780], 'Color', 'w', 'Visible', 'off');

    for k = 1:numel(frame_steps)
        step = frame_steps(k);
        t = step * params.algorithm.dt;
        plume_state = update_plume(t, params);
        C_max = max(plume_state.C(:));

        clf(fig);
        ax = axes(fig);
        hold(ax, 'on');

        if C_max > 0
            th_outer = C_max * params.plume.boundary_threshold;
            th_mid = C_max * 0.10;
            th_core = C_max * 0.28;

            [faces_o, verts_o] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_outer);
            if ~isempty(faces_o)
                patch(ax, 'Faces', faces_o, 'Vertices', verts_o, ...
                      'FaceAlpha', 0.12, 'FaceColor', [1.00 0.78 0.18], 'EdgeColor', 'none', ...
                      'DisplayName', '扩散边界');
            end

            [faces_m, verts_m] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_mid);
            if ~isempty(faces_m)
                patch(ax, 'Faces', faces_m, 'Vertices', verts_m, ...
                      'FaceAlpha', 0.18, 'FaceColor', [1.00 0.48 0.05], 'EdgeColor', 'none', ...
                      'DisplayName', '中浓度羽流');
            end

            [faces_c, verts_c] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_core);
            if ~isempty(faces_c)
                patch(ax, 'Faces', faces_c, 'Vertices', verts_c, ...
                      'FaceAlpha', 0.30, 'FaceColor', [0.90 0.05 0.00], 'EdgeColor', 'none', ...
                      'DisplayName', '高浓度核心');
            end

            [~, yidx] = min(abs(plume_state.y - params.plume.source_pos(2)));
            X_slice = squeeze(plume_state.X(:, yidx, :));
            Z_slice = squeeze(plume_state.Z(:, yidx, :));
            C_slice = squeeze(plume_state.C(:, yidx, :));
            C_slice(C_slice < th_outer) = NaN;
            surf(ax, X_slice, params.plume.source_pos(2) * ones(size(X_slice)), Z_slice, C_slice, ...
                 'FaceAlpha', 0.50, 'EdgeColor', 'none', 'FaceColor', 'interp', ...
                 'HandleVisibility', 'off');
        end

        scatter3(ax, params.plume.source_pos(1), params.plume.source_pos(2), params.plume.source_pos(3), ...
                 360, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
                 'DisplayName', '溢油源');

        front_x = min(domain.xmax, params.plume.source_pos(1) + params.plume.front_initial_extent + params.plume.u_current * t);
        plot3(ax, [front_x front_x], [domain.ymin domain.ymax], [params.plume.source_pos(3) params.plume.source_pos(3)], ...
              'k--', 'LineWidth', 1.3, 'DisplayName', '扩散前沿');

        for i = 1:n_agents
            traj_i = reshape(trajectory(1:step, i, :), step, 3);
            if i <= 4
                display_name = sprintf('AUV %d轨迹', i);
                handle_visibility = 'on';
            else
                display_name = '';
                handle_visibility = 'off';
            end
            plot3(ax, traj_i(:,1), traj_i(:,2), traj_i(:,3), '-', ...
                  'Color', colors(i,:), 'LineWidth', 1.8, ...
                  'DisplayName', display_name, 'HandleVisibility', handle_visibility);
            scatter3(ax, traj_i(end,1), traj_i(end,2), traj_i(end,3), ...
                     125, colors(i,:), 's', 'filled', 'MarkerEdgeColor', 'k', ...
                     'LineWidth', 1.2, 'HandleVisibility', 'off');
        end

        hold(ax, 'off');
        axis(ax, [domain.xmin domain.xmax domain.ymin domain.ymax domain.zmin domain.zmax]);
        daspect(ax, [1 1 1]);
        view(ax, 42, 24);
        grid(ax, 'on');
        colormap(ax, turbo);
        clim(ax, [0 max(C_max, eps)]);
        camlight(ax, 'headlight');
        lighting(ax, 'gouraud');
        xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
        title(ax, sprintf('动态溢油源扩散与AUV实时监测  t = %.0f s', t), 'FontSize', 14);
        legend(ax, 'Location', 'northeastoutside', 'FontSize', 7);
        set(ax, 'FontSize', 10);
        drawnow;

        frame = getframe(fig);
        [im, map] = rgb2ind(frame2im(frame), 256);
        if k == 1
            imwrite(im, map, save_path, 'gif', 'LoopCount', Inf, 'DelayTime', 0.20);
        else
            imwrite(im, map, save_path, 'gif', 'WriteMode', 'append', 'DelayTime', 0.20);
        end
    end

    close(fig);
end
