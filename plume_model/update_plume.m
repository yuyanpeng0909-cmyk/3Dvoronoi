function plume_state = update_plume(t, params)
% update_plume - 生成当前时刻的羽流浓度场三维采样网格
%
% 输入:
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   plume_state: 结构体，包含网格坐标和浓度场
%     .X, .Y, .Z: 三维网格坐标 (ndgrid格式)
%     .C: 浓度场 (同维度)
%     .x, .y, .z: 一维坐标向量

    nx = 80; ny = 60; nz = 50;
    x = linspace(params.domain.xmin, params.domain.xmax, nx);
    y = linspace(params.domain.ymin, params.domain.ymax, ny);
    z = linspace(params.domain.zmin, params.domain.zmax, nz);
    [X, Y, Z] = ndgrid(x, y, z);

    C = gaussian_plume_3d(X, Y, Z, t, params);

    plume_state = struct(...
        'X', X, 'Y', Y, 'Z', Z, ...
        'C', C, ...
        'x', x, 'y', y, 'z', z ...
    );
end
