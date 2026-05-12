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
    frame_count = 32;
    frame_steps = unique(round(linspace(1, steps, frame_count)));
    domain = params.domain;

    fig = figure('Name', '动态溢油扩散与AUV监测GIF', ...
                 'Position', [100 100 980 760], 'Color', 'w', 'Visible', 'off');

    for k = 1:numel(frame_steps)
        step = frame_steps(k);
        t = step * params.algorithm.dt;
        plume_state = update_plume(t, params);
        C_max = max(plume_state.C(:));

        clf(fig);
        hold on;

        if C_max > 0
            th_outer = C_max * params.plume.boundary_threshold;
            th_mid = C_max * 0.12;
            th_core = C_max * 0.32;

            [faces_o, verts_o] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_outer);
            if ~isempty(faces_o)
                patch('Faces', faces_o, 'Vertices', verts_o, ...
                      'FaceAlpha', 0.10, 'FaceColor', [1.00 0.72 0.20], 'EdgeColor', 'none', ...
                      'DisplayName', '扩散边界');
            end

            [faces_m, verts_m] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_mid);
            if ~isempty(faces_m)
                patch('Faces', faces_m, 'Vertices', verts_m, ...
                      'FaceAlpha', 0.16, 'FaceColor', [1.00 0.48 0.05], 'EdgeColor', 'none', ...
                      'DisplayName', '中浓度羽流');
            end

            [faces_c, verts_c] = isosurface(plume_state.X, plume_state.Y, plume_state.Z, plume_state.C, th_core);
            if ~isempty(faces_c)
                patch('Faces', faces_c, 'Vertices', verts_c, ...
                      'FaceAlpha', 0.25, 'FaceColor', [0.90 0.05 0.00], 'EdgeColor', 'none', ...
                      'DisplayName', '高浓度核心');
            end
        end

        scatter3(params.plume.source_pos(1), params.plume.source_pos(2), params.plume.source_pos(3), ...
                 330, 'r', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
                 'DisplayName', '溢油源');

        for i = 1:n_agents
            traj_i = reshape(trajectory(1:step, i, :), step, 3);
            plot3(traj_i(:,1), traj_i(:,2), traj_i(:,3), '-', ...
                  'Color', colors(i,:), 'LineWidth', 1.8, ...
                  'DisplayName', sprintf('AUV %d轨迹', i));
            scatter3(traj_i(end,1), traj_i(end,2), traj_i(end,3), ...
                     125, colors(i,:), 's', 'filled', 'MarkerEdgeColor', 'k', ...
                     'LineWidth', 1.2, 'HandleVisibility', 'off');
        end

        hold off;
        axis([domain.xmin domain.xmax domain.ymin domain.ymax domain.zmin domain.zmax]);
        axis equal;
        view(42, 24);
        grid on;
        camlight('headlight');
        lighting gouraud;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title(sprintf('动态溢油源扩散与AUV实时监测  t = %.0f s', t), 'FontSize', 14);
        legend('Location', 'northeastoutside', 'FontSize', 7);
        set(gca, 'FontSize', 10);
        drawnow;

        frame = getframe(fig);
        [im, map] = rgb2ind(frame2im(frame), 256);
        if k == 1
            imwrite(im, map, save_path, 'gif', 'LoopCount', Inf, 'DelayTime', 0.22);
        else
            imwrite(im, map, save_path, 'gif', 'WriteMode', 'append', 'DelayTime', 0.22);
        end
    end

    close(fig);
end
