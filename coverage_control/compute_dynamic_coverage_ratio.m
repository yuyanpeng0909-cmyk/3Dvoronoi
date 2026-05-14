function ratio = compute_dynamic_coverage_ratio(agent_positions, params, plume_state)
% compute_dynamic_coverage_ratio - 计算动态覆盖率
% 羽流体积中被至少一个AUV感知范围覆盖的比例
%
% CR(t) = |{x : C(x,t) >= threshold AND min_i ||x - p_i|| <= R_s}|
%       / |{x : C(x,t) >= threshold}|
%
% 输入:
%   agent_positions: n x 3 AUV位置
%   params: 参数结构体
%   plume_state: 可选，update_plume返回的结构体（避免重复计算）
%
% 输出:
%   ratio: 0~1之间的覆盖率

    if nargin < 3 || isempty(plume_state)
        error('需要提供plume_state参数');
    end

    C = plume_state.C;
    C_max = max(C(:));
    if C_max <= 0
        ratio = 0;
        return;
    end

    threshold = params.plume.boundary_threshold * C_max;
    plume_mask = C >= threshold;
    n_plume = nnz(plume_mask);

    if n_plume == 0
        ratio = 0;
        return;
    end

    % 提取羽流内网格点坐标
    plume_points = [plume_state.X(plume_mask), ...
                    plume_state.Y(plume_mask), ...
                    plume_state.Z(plume_mask)];

    % 子采样以控制计算量
    max_pts = 5000;
    if size(plume_points, 1) > max_pts
        idx = randperm(size(plume_points, 1), max_pts);
        plume_points = plume_points(idx, :);
    end

    % 计算到最近AUV的距离
    min_dist = min(pdist2(plume_points, agent_positions), [], 2);

    % 感知范围内的比例
    ratio = mean(min_dist <= params.agent.sense_radius);
end
