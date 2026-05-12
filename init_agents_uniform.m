function positions = init_agents_uniform(params)
% init_agents_uniform - 在域内均匀网格分布智能体
%
% 输入:
%   params: 参数结构体
%
% 输出:
%   positions: n x 3 矩阵，均匀网格选取的智能体位置

    n = params.agent.num;
    domain = params.domain;

    % 生成近似均匀的三维网格
    nx = ceil(n^(1/3));
    ny = nx;
    nz = ceil(n / (nx * ny));

    % 在域的80%范围内均匀分布（避免紧贴边界）
    x = linspace(domain.xmin * 0.8, domain.xmax * 0.8, nx);
    y = linspace(domain.ymin * 0.8, domain.ymax * 0.8, ny);
    z = linspace(domain.zmin * 0.8, domain.zmax * 0.8, nz);
    [X, Y, Z] = ndgrid(x, y, z);
    grid_points = [X(:), Y(:), Z(:)];

    % 随机选取n个网格点
    idx = randperm(size(grid_points, 1), n);
    positions = grid_points(idx, :);
end
