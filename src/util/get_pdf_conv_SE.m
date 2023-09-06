function [pdf_conv, time_link] = get_pdf_conv_SE(pdf_link)
% get_pdf_conv_SE Compute convolution of PDF to concatenate links
%
% Input
%   pdf_link : Mean and covariance of PDF in SE(2)/SE(3) for each link 
%              distal end
%
% Output
%   pdf_conv : Mean and covariance of PDF for each link distal end after 
%              convolution in SE(2)/SE(3)
%   time_link: Time spent on computing each convolution
%
% Author
%   Sipu Ruan, 2022

pdf_conv = cell(1,length(pdf_link));
time_link = zeros(1,length(pdf_conv));

pdf_conv{1} = pdf_link{1};
for i = 2:length(pdf_conv)
    ti = tic;
    pdf_conv{i}.mean = pdf_conv{i-1}.mean * pdf_link{i}.mean;
    pdf_conv{i}.cov  = cov_SE_1_2(pdf_conv{i-1}.cov, pdf_link{i}.cov,...
        pdf_conv{i-1}.mean, pdf_link{i}.mean);
    time_link(i) = toc(ti);
end