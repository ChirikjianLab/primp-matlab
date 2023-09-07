function plot_gaussian_pdf(mean, cov)
[R, D] = eig(cov(4:6,4:6));
semi = diag(D.^(1/2));

[xx, yy, zz] = ellipsoid(0, 0, 0, semi(1), semi(2), semi(3));

x(1,:) = reshape(xx, 1, numel(xx));
x(2,:) = reshape(yy, 1, numel(yy));
x(3,:) = reshape(zz, 1, numel(zz));

x_r = R * x + mean(1:3,4);

size_e = size(xx);
xx_r = reshape(x_r(1,:), size_e);
yy_r = reshape(x_r(2,:), size_e);
zz_r = reshape(x_r(3,:), size_e);

surf(xx_r, yy_r, zz_r, 'EdgeColor', 'none', 'FaceAlpha', 0.3);

end