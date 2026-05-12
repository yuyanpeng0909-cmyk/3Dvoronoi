function plume_volume = estimate_plume_extent(t, params)
% estimate_plume_extent - 用网格阈值估计当前羽流有效体积
%
% 输入:
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   plume_volume: 浓度高于边界阈值的体积估计 (m^3)

    plume_state = update_plume(t, params);
    C_max = max(plume_state.C(:));
    if C_max <= 0
        plume_volume = 0;
        return;
    end

    threshold = params.plume.boundary_threshold * C_max;
    dx = plume_state.x(2) - plume_state.x(1);
    dy = plume_state.y(2) - plume_state.y(1);
    dz = plume_state.z(2) - plume_state.z(1);
    cell_volume = dx * dy * dz;
    plume_volume = nnz(plume_state.C >= threshold) * cell_volume;
end
