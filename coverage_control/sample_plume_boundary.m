function [boundary_points, assigned_agent, boundary_distance] = sample_plume_boundary(agent_positions, t, params)
% sample_plume_boundary - 采样当前溢油羽流扩散边界并均衡分配给所有智能体
%
% 输入:
%   agent_positions: n x 3 智能体位置
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   boundary_points: Nb x 3 边界附近采样点
%   assigned_agent: Nb x 1 智能体编号
%   boundary_distance: 平均智能体到所属边界点距离

    N = params.algorithm.boundary_sample_num;
    domain = params.domain;
    src = params.plume.source_pos;
    tau = max(t, 0);
    n_agents = size(agent_positions, 1);

    center_x = src(1) + params.plume.u_current * tau;
    center_z = min(0, src(3) + params.plume.w_buoyancy * tau);
    sx = min(max(sqrt(params.plume.sigma_x0^2 + 2 * params.plume.diffusion_x * tau) + 170, 140), 320);
    sy = min(max(sqrt(params.plume.sigma_y0^2 + 2 * params.plume.diffusion_y * tau) + 130, 110), 240);
    sz = min(max(sqrt(params.plume.sigma_z0^2 + 2 * params.plume.diffusion_z * tau) + 90, 75), 180);

    samples = zeros(N, 3);
    samples(:,1) = center_x + sx * randn(N, 1);
    samples(:,2) = src(2) + sy * randn(N, 1);
    samples(:,3) = center_z + sz * randn(N, 1);
    samples(:,1) = max(domain.xmin, min(domain.xmax, samples(:,1)));
    samples(:,2) = max(domain.ymin, min(domain.ymax, samples(:,2)));
    samples(:,3) = max(domain.zmin, min(domain.zmax, samples(:,3)));

    C = gaussian_plume_3d(samples(:,1), samples(:,2), samples(:,3), t, params);
    C_ref = max(max(C), params.plume.C_max_estimate * exp(-params.plume.decay_rate * tau));
    threshold = params.plume.boundary_threshold * C_ref;
    band = max(threshold * 0.45, eps);
    mask = abs(C - threshold) <= band & C > 0;

    if nnz(mask) < n_agents * 8
        [~, order] = sort(abs(C - threshold), 'ascend');
        keep_num = min(max(n_agents * 16, 80), numel(order));
        mask = false(N, 1);
        mask(order(1:keep_num)) = true;
    end

    boundary_points = samples(mask, :);
    if isempty(boundary_points)
        assigned_agent = zeros(0, 1);
        boundary_distance = NaN;
        return;
    end

    plume_center = [center_x, src(2), center_z];
    rel_agents = agent_positions - plume_center;
    agent_theta = atan2(rel_agents(:,3) ./ max(sz, eps), rel_agents(:,2) ./ max(sy, eps));
    [~, agent_order] = sort(agent_theta, 'ascend');

    rel_y = (boundary_points(:,2) - src(2)) ./ max(sy, eps);
    rel_z = (boundary_points(:,3) - center_z) ./ max(sz, eps);
    theta = atan2(rel_z, rel_y);
    [~, order_theta] = sort(theta, 'ascend');

    assigned_agent = zeros(size(boundary_points, 1), 1);
    for rank = 1:n_agents
        idx_start = floor((rank - 1) * numel(order_theta) / n_agents) + 1;
        idx_end = floor(rank * numel(order_theta) / n_agents);
        assigned_agent(order_theta(idx_start:idx_end)) = agent_order(rank);
    end

    dist_to_assigned = sqrt(sum((boundary_points - agent_positions(assigned_agent, :)).^2, 2));
    boundary_distance = mean(dist_to_assigned);
end
