function rmse = compute_boundary_rmse(agent_positions, params, plume_state)
% compute_boundary_rmse - 计算AUV到羽流等值面的边界追踪RMSE
%
% e_b(t) = sqrt( (1/n) * sum_i min_{x in S} ||p_i(t) - x||^2 )
% 其中 S = {x : C(x,t) ≈ eta * C_max}
%
% 输入:
%   agent_positions: n x 3 AUV位置
%   params: 参数结构体
%   plume_state: update_plume返回的结构体
%
% 输出:
%   rmse: 边界追踪均方根误差 (m)

    if nargin < 3 || isempty(plume_state)
        error('需要提供plume_state参数');
    end

    C = plume_state.C;
    C_max = max(C(:));
    if C_max <= 0
        rmse = NaN;
        return;
    end

    threshold = params.plume.boundary_threshold * C_max;

    % 在阈值带宽内找边界点
    band = max(threshold * 0.5, eps);
    boundary_mask = abs(C - threshold) <= band & C > 0;

    if ~any(boundary_mask(:))
        rmse = NaN;
        return;
    end

    boundary_points = [plume_state.X(boundary_mask), ...
                       plume_state.Y(boundary_mask), ...
                       plume_state.Z(boundary_mask)];

    % 子采样以控制计算量
    max_pts = 800;
    if size(boundary_points, 1) > max_pts
        idx = randperm(size(boundary_points, 1), max_pts);
        boundary_points = boundary_points(idx, :);
    end

    % 对每个AUV计算到最近边界点的距离
    n = size(agent_positions, 1);
    min_dists = zeros(n, 1);
    dist_matrix = pdist2(agent_positions, boundary_points);
    min_dists = min(dist_matrix, [], 2);

    rmse = sqrt(mean(min_dists.^2));
end
