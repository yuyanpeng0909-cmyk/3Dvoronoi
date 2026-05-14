function C = gaussian_plume_3d(x, y, z, t, params)
% gaussian_plume_3d - 持续泄漏的三维对流-扩散-衰减羽流浓度场
%
% 输入:
%   x, y, z: 标量或同维数组，空间坐标 (m)
%   t: 当前时间 (s)
%   params: 参数结构体（由 init_parameters 生成）
%
% 输出:
%   C: 浓度值 (kg/m^3)，与输入同维度

    src = params.plume.source_pos;
    Q = params.plume.Q;
    u = params.plume.u_current;
    w = params.plume.w_buoyancy;
    sx0 = params.plume.sigma_x0;
    sy0 = params.plume.sigma_y0;
    sz0 = params.plume.sigma_z0;
    Dx = params.plume.diffusion_x;
    Dy = params.plume.diffusion_y;
    Dz = params.plume.diffusion_z;
    lambda = params.plume.decay_rate;
    n_release = params.plume.release_substeps;

    tau = max(t, 0);
    C = zeros(size(x));
    if tau <= 0
        return;
    end

    release_times = linspace(0, tau, n_release + 1);
    release_times = 0.5 * (release_times(1:end-1) + release_times(2:end));
    dtau = tau / n_release;

    for r = release_times
        age = tau - r;
        center_x = src(1) + u * age;
        center_y = src(2);
        center_z = min(0, src(3) + w * age);

        sigma_x = sqrt(sx0^2 + 2 * Dx * age);
        sigma_y = sqrt(sy0^2 + 2 * Dy * age);
        sigma_z = sqrt(sz0^2 + 2 * Dz * age);

        kernel = exp(-((x - center_x).^2) ./ (2 * sigma_x^2) ...
                     -((y - center_y).^2) ./ (2 * sigma_y^2));
        vertical = exp(-((z - center_z).^2) ./ (2 * sigma_z^2)) + ...
                   exp(-((z + center_z).^2) ./ (2 * sigma_z^2));
        downstream_gate = 1 ./ (1 + exp(-(x - src(1)) ./ params.plume.front_smoothing));
        amplitude = Q * dtau * exp(-lambda * age) / ((2*pi)^(3/2) * sigma_x * sigma_y * sigma_z);
        C = C + amplitude .* kernel .* vertical .* downstream_gate;
    end

    C = max(C, 0);
end
