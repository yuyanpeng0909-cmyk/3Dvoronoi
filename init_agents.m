function positions = init_agents(params)
% init_agents - 在溢油源附近按监测环初始化智能体位置
%
% 输入:
%   params: 参数结构体
%
% 输出:
%   positions: n x 3 矩阵，每行为一个智能体的 [x, y, z] 坐标

    n = params.agent.num;
    domain = params.domain;
    src = params.plume.source_pos;
    theta = linspace(0, 2*pi, n + 1)';
    theta(end) = [];

    x0 = src(1) + 45;
    ry = min(0.75 * params.agent.sense_radius, 0.35 * (domain.ymax - domain.ymin));
    rz = min(0.55 * params.agent.sense_radius, 0.25 * (domain.zmax - domain.zmin));

    positions = zeros(n, 3);
    positions(:,1) = x0 + 12 * cos(theta + pi / n);
    positions(:,2) = src(2) + ry * cos(theta);
    positions(:,3) = src(3) + rz * sin(theta);

    positions(:,1) = max(domain.xmin, min(domain.xmax, positions(:,1)));
    positions(:,2) = max(domain.ymin, min(domain.ymax, positions(:,2)));
    positions(:,3) = max(domain.zmin, min(domain.zmax, positions(:,3)));
end
