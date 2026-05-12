function phi = compute_density(x, y, z, t, params)
% compute_density - 计算给定位置和时间的密度函数值
%
% 输入:
%   x, y, z: 标量或同维数组，空间坐标 (m)
%   t: 当前时间 (s)
%   params: 参数结构体
%
% 输出:
%   phi: 密度值，与输入同维度，处处 > 0
%
% 数学模型:
%   φ(x,t) = α · C_norm(x,t) + β
%   C_norm = min(C / C_max_est, 1) ∈ [0, 1]
%   保证 φ ≥ β > 0（全域正密度）

    % 计算浓度
    C = gaussian_plume_3d(x, y, z, t, params);

    % 归一化到 [0, 1]
    C_max = params.plume.C_max_estimate;
    C_norm = min(C / C_max, 1);

    % 密度函数
    alpha = params.density.alpha;
    beta = params.density.beta;
    phi = alpha * C_norm + beta;
end
