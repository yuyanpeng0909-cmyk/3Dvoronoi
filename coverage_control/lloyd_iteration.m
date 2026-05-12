function [new_positions, centroids, move_distance, sample_data] = lloyd_iteration(agent_positions, t, prev_centroids, params)
% lloyd_iteration - 三维Voronoi覆盖控制单步迭代（含质心前馈与边界追踪）
%
% 输入:
%   agent_positions: n x 3 当前位置
%   t: 当前时间 (s)
%   prev_centroids: n x 3 上一时刻质心（空矩阵则无前馈）
%   params: 参数结构体
%
% 输出:
%   new_positions: n x 3 更新后位置
%   centroids: n x 3 当前质心
%   move_distance: 平均移动距离 (m)
%   sample_data: struct 采样数据（供 coverage_quality 和监测指标复用）

    n = size(agent_positions, 1);
    k = params.agent.control_gain;
    v_max = params.agent.max_speed;
    dt = params.algorithm.dt;
    gamma_ff = params.algorithm.ff_gain;

    [centroids, samples, phi, dist_matrix, nearest_agent, g] = compute_centroid(agent_positions, t, params);

    v_feedback = k * (centroids - agent_positions);

    if ~isempty(prev_centroids)
        dcen_dt = (centroids - prev_centroids) / dt;
        v_feedforward = gamma_ff * dcen_dt;
    else
        v_feedforward = zeros(n, 3);
    end

    [boundary_points, boundary_assign, boundary_distance] = sample_plume_boundary(agent_positions, t, params);
    boundary_targets = agent_positions;
    for i = 1:n
        mask = boundary_assign == i;
        if any(mask)
            boundary_targets(i, :) = mean(boundary_points(mask, :), 1);
        end
    end
    v_boundary = params.algorithm.boundary_gain * (boundary_targets - agent_positions);

    plume_center = [params.plume.source_pos(1) + 0.55 * params.plume.u_current * max(t, 0), ...
                    params.plume.source_pos(2), params.plume.source_pos(3)];
    rel = agent_positions - plume_center;
    desired_spacing = 0.75 * params.agent.sense_radius;
    v_separation = zeros(n, 3);
    for i = 1:n
        for j = 1:n
            if i == j
                continue;
            end
            diff_ij = rel(i, :) - rel(j, :);
            dist_ij = norm(diff_ij);
            if dist_ij > eps && dist_ij < desired_spacing
                v_separation(i, :) = v_separation(i, :) + ...
                    0.18 * (desired_spacing - dist_ij) / desired_spacing * diff_ij / dist_ij;
            end
        end
    end

    velocity = v_feedback + v_feedforward + v_boundary + v_separation;

    speed = sqrt(sum(velocity.^2, 2));
    too_fast = speed > v_max;
    if any(too_fast)
        velocity(too_fast, :) = v_max * velocity(too_fast, :) ./ speed(too_fast);
    end

    new_positions = agent_positions + velocity * dt;

    domain = params.domain;
    new_positions(:,1) = max(domain.xmin, min(domain.xmax, new_positions(:,1)));
    new_positions(:,2) = max(domain.ymin, min(domain.ymax, new_positions(:,2)));
    new_positions(:,3) = max(domain.zmin, min(domain.zmax, new_positions(:,3)));

    move_distance = mean(sqrt(sum((new_positions - agent_positions).^2, 2)));

    w = phi ./ max(g, 1e-20);
    sample_data = struct('samples', samples, 'phi', phi, ...
                         'dist_matrix', dist_matrix, 'nearest_agent', nearest_agent, ...
                         'g', g, 'w', w, ...
                         'boundary_points', boundary_points, ...
                         'boundary_assign', boundary_assign, ...
                         'boundary_distance', boundary_distance);
end
