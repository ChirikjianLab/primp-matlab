function pdf_link = get_pdf_from_pose(pose, group_name)
% get_pdf_from_pose Compute PDF mean and covariance for each link of
% the robot
%
% Input
%   pose: sampled poses of each link distal end, a cell structure
%
% Output
%   pdf_link: Mean and covariance of PDF in Lie group for each link 
%             distal end
%
% Author
%   Sipu Ruan, 2023
%
% Dependency
%   get_mean_cov

n_link = length(pose);
pdf_link = cell(1, n_link);
for i = 1:n_link
    [pdf_link{i}.mean, pdf_link{i}.cov, flag] = get_mean_cov(...
            pose{i}.matrix, group_name);
    
    if ~flag
        pdf_link{i}.mean_exp = mean(pose{i}.exponential, 2);
        pdf_link{i}.mean = get_exp_mapping(pdf_link{i}.mean_exp,...
            group_name);
        pdf_link{i}.cov = cov(pose{i}.exponential');
    end
end