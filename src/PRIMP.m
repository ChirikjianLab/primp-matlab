classdef PRIMP < handle
    % PRIMP class for PRobabilisticall-Informed Motion Primitives
    %
    %  Author
    %    Sipu Ruan, 2022

    properties
        % Initial mean trajectory with covariance
        mean_init
        cov_init

        % Viewing frame
        h_view = eye(4)

        % Mean of joint distribution
        mean_joint

        % Covariance of joint distribution
        cov_joint
        cov_joint_inv

        % Via pose info
        t_via = 0
        g_via
        cov_via
        g_via_view
        cov_via_view

        % Time step index for via pose
        t_idx = 0
        t_idx_matrix

        % Samples from distribution
        g_samples

        % Parameters
        n_step = 0
        n_sample = 20
        group_name = 'SE'
    end

    methods
        %% Constructor
        function obj = PRIMP(mean_init, cov_init, param)
            obj.n_step = size(mean_init, 3);
            obj.mean_init = mean_init;
            obj.cov_init = cov_init;

            obj.mean_joint = obj.mean_init;
            obj.cov_joint = eye(6 * obj.n_step);
            obj.cov_joint_inv = eye(6 * obj.n_step);

            obj.t_idx_matrix = zeros(6, 6*obj.n_step);

            if nargin == 3
                obj.n_sample = param.n_sample;
                obj.group_name = param.group_name;
            end

            obj.g_samples = cell(obj.n_sample, 1);
            for k = 1:obj.n_sample
                % Pre-allocate memory for sampled poses
                obj.g_samples{k} = nan(4,4,obj.n_step);
            end

            % Compute joint PDF from individual poses
            obj.compute_joint_pdf();
        end

        %% Getter functions
        % Get covariance of the joint Gaussian PDF
        function cov_joint = get_joint_pdf(obj)
            cov_joint = obj.cov_joint;
        end

        % Get mean/covariance conditioned by new pose
        function [mean_cond, cov_cond] = get_condition_pdf(obj, t_via,...
                g_via, cov_via)
            obj.t_via = t_via;
            obj.g_via = g_via;
            obj.cov_via = cov_via;

            % Change new view frame for via point
            obj.g_via_view = get_conjugation(obj.g_via, inv(obj.h_view),...
                obj.group_name);
            obj.cov_via_view = Adjoint(obj.h_view, obj.group_name) \...
                obj.cov_via / Adjoint(obj.h_view, obj.group_name)';

            if nargin == 3
                obj.cov_via = zeros(6,6);
            end

            % Compute conditional probability on via point
            obj.compute_time_step_idx();
            obj.condition_on_via_point();

            mean_cond = obj.mean_joint;
            cov_cond = obj.cov_joint;
        end

        % Get distribution after fusing with workspace density
        function [mean_fused, cov_fused] = get_fusion_workspace_density(...
                obj, mean_wd, cov_wd)
            obj.fuse_with_workspace_density(mean_wd, cov_wd);

            mean_fused = obj.mean_joint;
            cov_fused = obj.cov_joint;
        end

        % Get samples from the computed joint/conditional PDF
        function g_samples = get_samples(obj)
            obj.generate_samples_from_joint_pdf();

            g_samples = obj.g_samples();
        end

        %% Setter functions
        % Set new frame of view
        function set_new_view_frame(obj, h)
            obj.h_view = h;

            % Change of view for the mean trajectory
            for i = 1:obj.n_step
                obj.mean_init(:,:,i) = get_conjugation(...
                    obj.mean_init(:,:,i), inv(obj.h_view), obj.group_name);
                obj.cov_init(:,:,i) =...
                    Adjoint(obj.h_view, obj.group_name) \...
                    obj.cov_init(:,:,i) /...
                    Adjoint(obj.h_view, obj.group_name)';
            end

            obj.mean_joint = obj.mean_init;

            % Compute joint PDF from individual poses
            obj.compute_joint_pdf();
        end
    end

    methods (Access = private)
        %% Compute covariance of the joint Gaussian PDF
        function compute_joint_pdf(obj)
            % Inverse of the joint covariance given fixed start/goal poses
            for i = 1:obj.n_step
                idx_block = (i-1)*6+1:i*6;

                % Fill in the blocks of the joint covariance
                if i ~= obj.n_step
                    % Pose difference between adjacent steps
                    d_mu_i = obj.mean_init(:,:,i)\obj.mean_init(:,:,i+1);
                    adjoint_mat = Adjoint(d_mu_i, obj.group_name);
                    cov_tilde = adjoint_mat * obj.cov_init(:,:,i+1)...
                        * adjoint_mat';

                    obj.cov_joint_inv(idx_block, idx_block) =...
                        inv( obj.cov_init(:,:,i) ) + inv( cov_tilde );
                    obj.cov_joint_inv(idx_block, idx_block+6) = -inv(...
                        obj.cov_init(:,:,i+1) * adjoint_mat' );
                    obj.cov_joint_inv(idx_block+6, idx_block) = -inv(...
                        adjoint_mat * obj.cov_init(:,:,i+1) );
                else
                    obj.cov_joint_inv(idx_block, idx_block) =...
                        inv( obj.cov_init(:,:,i) );
                end
            end

            % Ensure it is symmetric
            obj.cov_joint_inv =...
                (obj.cov_joint_inv + obj.cov_joint_inv') / 2;

            obj.cov_joint = inv(obj.cov_joint_inv);
        end

        %% Compute time step index
        function compute_time_step_idx(obj)
            % Locate the closest pose step to the requested via pose
            obj.t_idx = floor(obj.t_via * obj.n_step);
            if obj.t_idx <= 0
                obj.t_idx = 1;
            elseif obj.t_idx >= obj.n_step
                obj.t_idx = obj.n_step;
            end

            % Compute time index in matrix form
            P = zeros(1, obj.n_step);
            P(obj.t_idx) = 1;
            obj.t_idx_matrix = kron(P, eye(6));
        end

        %% Compute conditional Gaussain PDF given a via point
        function condition_on_via_point(obj)
            k_gain = obj.cov_joint * obj.t_idx_matrix' /...
                (obj.t_idx_matrix * obj.cov_joint * obj.t_idx_matrix' +...
                obj.cov_via_view);

            % Mean after conditioning: condition on the desired pose g_via
            x_mu = k_gain * get_exp_coord(obj.mean_joint(:,:,obj.t_idx)\...
                obj.g_via_view, obj.group_name);

            % Update covariance
            obj.cov_joint = obj.cov_joint -...
                k_gain * obj.t_idx_matrix * obj.cov_joint;

            % Compute trajectory after conditioning
            for i = 1:obj.n_step
                obj.mean_joint(:,:,i) = ... 
                    obj.mean_joint(:,:,i) *...
                    get_exp_mapping(x_mu(6*(i-1)+1:6*i, 1),...
                    obj.group_name);
            end
        end

        %% Fusion with robot-specific workspace density
        function fuse_with_workspace_density(obj, mean_wd, cov_wd)
            cov_wd_traj = kron(eye(obj.n_step), cov_wd);

            x_wd_traj = zeros(obj.n_step*6, 1);
            for i = 1:obj.n_step
                idx_block = (i-1)*6+1:i*6;
                x_wd_traj(idx_block) = get_exp_coord(...
                    obj.mean_joint(:,:,i) \ mean_wd);
            end

            % Mean and covariance updates
            k_gain = obj.cov_joint / (obj.cov_joint + cov_wd_traj);

            x_fused = k_gain * x_wd_traj;
            for i = 1:obj.n_step
                obj.mean_joint(:,:,i) = obj.mean_joint(:,:,i) *...
                    get_exp_mapping(x_fused(6*(i-1)+1:6*i, 1),...
                    obj.group_name);
            end
            obj.cov_joint = (eye(size(k_gain,1)) - k_gain) * obj.cov_joint;
        end


        %% Generate samples from joint PDF
        function generate_samples_from_joint_pdf(obj)
            obj.g_samples = cell(obj.n_sample, 1);

            % Generate samples from joint distribution
            X = zeros(6*obj.n_step, 1);
            x_sample = mvnrnd(X, obj.cov_joint, obj.n_sample)';

            for k = 1:obj.n_sample
                obj.g_samples{k} = zeros(4, 4, obj.n_step);
                for j = 1:obj.n_step
                    obj.g_samples{k}(:,:,j) =...
                        obj.mean_joint(:,:,j) *...
                        get_exp_mapping(x_sample(6*(j-1)+1:6*(j), k),...
                        obj.group_name);
                end
            end
        end

    end
end
