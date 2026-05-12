function positions = init_agents(params)
% init_agents - 在域内随机生成智能体初始位置
%
% 输入:
%   params: 参数结构体
%
% 输出:
%   positions: n x 3 矩阵，每行为一个智能体的 [x, y, z] 坐标

    n = params.agent.num;
    domain = params.domain;
    positions = zeros(n, 3);
    positions(:,1) = domain.xmin + (domain.xmax - domain.xmin) * rand(n, 1);
    positions(:,2) = domain.ymin + (domain.ymax - domain.ymin) * rand(n, 1);
    positions(:,3) = domain.zmin + (domain.zmax - domain.zmin) * rand(n, 1);
end
