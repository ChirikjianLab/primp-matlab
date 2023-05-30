function pdf_link = get_pdf_from_pose_SE(pose)
% get_pdf_from_pose_SE Compute PDF mean and covariance for each link of
% the robot
%
% Input
%   pose: sampled poses of each link distal end, a cell structure
%
% Output
%   pdf_link: Mean and covariance of PDF in SE(2)/SE(3) for each link 
%             distal end
%
% Author
%   Sipu Ruan, 2022

pdf_link = cell(1, length(pose.cartesian));
e_link = cell(1, length(pose.cartesian));
for i = 1:length(pose.cartesian)
%     num = size(pose.cartesian{i}, 2);
%     g = nan(3,3,num);
%     for j = 1:num
%         g(:,:,j) = [rot2(pose.cartesian{i}(3,j)), pose.cartesian{i}(1:2,j);
%             0,0,1];
%     end
% 
%     [pdf_link{i}.mean, pdf_link{i}.cov] = mean_cov_SE(g);
%     mu_exp = get_exp_coord(pdf_link{i}.mean);

    pts_exp = pose.exponential{i};

    pdf_link{i}.mean_exp = mean(pts_exp, 2);
    pdf_link{i}.mean = get_exp_mapping(pdf_link{i}.mean_exp);
    pdf_link{i}.cov = cov(pts_exp');
 
%     [R, Lambda] = eig(real(pdf_link{i}.cov^(1/2)));
%     a = diag(Lambda);
%     quat = rotm2quat(R);
% 
%     for k = 1:length(a)
%         a(a<1e-5) = 0.1;
%     end
% 
%     e_link{i} = Ellipsoid({a', [0,0], pdf_link{i}.mean, quat, [20,20]});

%     figure; hold on; axis equal;
%     plot3(pose.exponential{i}(1,:), pose.exponential{i}(2,:), pose.exponential{i}(3,:), 'k.')
% 
%     mean_end_exp = get_exp_coord(pdf_link{i}.mean);
%     exp_sample = mvnrnd(mean_end_exp, pdf_link{i}.cov, num_samples)';
%     plot3(exp_sample(1,:), exp_sample(2,:), exp_sample(3,:), 'r.')
%     e_link{i}.PlotShape('r', 0.3);
end