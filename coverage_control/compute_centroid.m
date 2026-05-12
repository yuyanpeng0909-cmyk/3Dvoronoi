function [centroids, samples, phi, dist_matrix, nearest_agent, g] = compute_centroid(agent_positions, t, params)
% compute_centroid - 基于重要性采样的三维Voronoi密度加权质心计算
%
% 输入:
%   agent_positions: n x 3 矩阵，智能体当前位置
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   centroids: n x 3 矩阵，每个Voronoi单元的密度加权质心
%   samples: N x 3 采样点
%   phi: N x 1 密度值
%   dist_matrix: N x n 距离矩阵
%   nearest_agent: N x 1 最近智能体索引
%   g: N x 1 采样密度

    n = size(agent_positions, 1);
    N = params.algorithm.sample_num;
    domain = params.domain;
    V_domain = (domain.xmax-domain.xmin) * (domain.ymax-domain.ymin) * (domain.zmax-domain.zmin);

    p_uniform = 0.45;
    N_uniform = round(N * p_uniform);
    N_plume = N - N_uniform;

    samples_u = zeros(N_uniform, 3);
    samples_u(:,1) = domain.xmin + (domain.xmax-domain.xmin) * rand(N_uniform, 1);
    samples_u(:,2) = domain.ymin + (domain.ymax-domain.ymin) * rand(N_uniform, 1);
    samples_u(:,3) = domain.zmin + (domain.zmax-domain.zmin) * rand(N_uniform, 1);

    src = params.plume.source_pos;
    tau = max(t, 0);
    center_x = src(1) + 0.55 * params.plume.u_current * tau;
    mean_x = min(domain.xmax, max(domain.xmin, center_x));
    sigma_x_sample = min(max(params.plume.sigma_x0 + sqrt(2 * params.plume.diffusion_x * tau) + 120, 100), 260);
    sigma_y_sample = min(max(params.plume.sigma_y0 + sqrt(2 * params.plume.diffusion_y * tau) + 90, 80), 210);
    sigma_z_sample = min(max(params.plume.sigma_z0 + sqrt(2 * params.plume.diffusion_z * tau) + 65, 55), 150);

    samples_p = zeros(N_plume, 3);
    samples_p(:,1) = mean_x + sigma_x_sample * randn(N_plume, 1);
    samples_p(:,2) = src(2) + sigma_y_sample * randn(N_plume, 1);
    samples_p(:,3) = src(3) + sigma_z_sample * randn(N_plume, 1);

    samples_p(:,1) = max(domain.xmin, min(domain.xmax, samples_p(:,1)));
    samples_p(:,2) = max(domain.ymin, min(domain.ymax, samples_p(:,2)));
    samples_p(:,3) = max(domain.zmin, min(domain.zmax, samples_p(:,3)));

    samples = [samples_u; samples_p];
    phi = compute_density(samples(:,1), samples(:,2), samples(:,3), t, params);

    g = zeros(N, 1);
    g(1:N_uniform) = p_uniform / V_domain;

    g_x = normpdf(samples_p(:,1), mean_x, sigma_x_sample);
    g_y = normpdf(samples_p(:,2), src(2), sigma_y_sample);
    g_z = normpdf(samples_p(:,3), src(3), sigma_z_sample);
    g(N_uniform+1:end) = (1-p_uniform) * g_x .* g_y .* g_z;
    g = max(g, 1e-20);

    w = phi ./ g;
    dist_matrix = pdist2(samples, agent_positions);
    [~, nearest_agent] = min(dist_matrix, [], 2);

    centroids = zeros(n, 3);
    for i = 1:n
        mask = (nearest_agent == i);
        if any(mask)
            w_i = w(mask);
            w_total = sum(w_i);
            if w_total > 0
                centroids(i, :) = sum(w_i .* samples(mask, :), 1) / w_total;
            else
                centroids(i, :) = agent_positions(i, :);
            end
        else
            centroids(i, :) = agent_positions(i, :);
        end
    end
end
