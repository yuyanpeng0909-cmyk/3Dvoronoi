function C = gaussian_plume_3d(x, y, z, t, params)
% gaussian_plume_3d - 三维时变高斯溢油羽流浓度场计算
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
    sx0 = params.plume.sigma_x0;
    sy0 = params.plume.sigma_y0;
    sz0 = params.plume.sigma_z0;
    kx = params.plume.kx;
    ky = params.plume.ky;
    kz = params.plume.kz;
    Dx = params.plume.diffusion_x;
    Dy = params.plume.diffusion_y;
    Dz = params.plume.diffusion_z;
    lambda = params.plume.decay_rate;

    tau = max(t, 0);
    dx = x - src(1);
    dy = y - src(2);
    dz = z - src(3);

    plume_front = src(1) + params.plume.front_initial_extent + u * tau;
    front_gate = 1 ./ (1 + exp((x - plume_front) ./ params.plume.front_smoothing));
    downstream_mask = dx >= 0;

    sigma_x = sx0 + kx * max(dx, 0) + sqrt(2 * Dx * tau);
    sigma_y = sy0 + ky * max(dx, 0) + sqrt(2 * Dy * tau);
    sigma_z = sz0 + kz * max(dx, 0) + sqrt(2 * Dz * tau);

    sigma_x = max(sigma_x, sx0);
    sigma_y = max(sigma_y, sy0);
    sigma_z = max(sigma_z, sz0);

    center_x = src(1) + 0.55 * u * tau;
    axial = exp(-((x - center_x).^2) ./ (2 * sigma_x.^2));
    lateral = exp(-(dy.^2) ./ (2 * sigma_y.^2));
    direct = exp(-(dz.^2) ./ (2 * sigma_z.^2));
    mirror = exp(-(z + src(3)).^2 ./ (2 * sigma_z.^2));

    source_gate = 1 - exp(-tau / 8);
    amplitude = Q .* source_gate .* exp(-lambda * tau) ./ ...
        ((2*pi)^(3/2) .* sigma_x .* sigma_y .* sigma_z);

    C = amplitude .* axial .* lateral .* (direct + mirror) .* front_gate;
    C(~downstream_mask) = 0;
    C = max(C, 0);
end
