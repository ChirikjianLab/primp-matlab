classdef KernalizedMovementPrimitives < handle
    % KernelizedMovementPrimitives Wrapper class for Orientation-KMP
    %
    % Author
    %   Sipu Ruan, 2022
    %
    % Reference
    %   https://github.com/yanlongtu/robInfLib-matlab

    properties
        % Demonstrated trajectories
        g_demo
        n_demo = 0
        n_step = 50

        % Reference trajectory distribution based on GMM
        traj_ref
        traj_ref_new
        n_step_new

        % GMM model
        model

        % KMP parameters
        lamda = 1;  % control mean prediction
        lamdac = 60; % control variance prediction
        kh = 10;

        % Trajectory predicted by KMP
        traj_kmp
    end

    properties (Access = private)
        dim = 6
        group_name = 'PCG';
    end

    methods
        function obj = KernalizedMovementPrimitives(g_demo, model, param)
            obj.g_demo = g_demo;
            obj.n_demo = length(g_demo);

            obj.model = model;

            if nargin > 1
                obj.n_step = param.n_step;

                obj.lamda = param.kmp_param.lamda;
                obj.lamdac = param.kmp_param.lamdac;
                obj.kh = param.kmp_param.kh;
            end

            % Number of variables
            % [t,w1,w2,w3,x1,x2,x3,aw1,aw2,aw3,v1,v2,v3]
            obj.model.nbVar = 1+2*obj.dim;

            % Learn GMM/GMR model
            obj.learn_gmm();
        end

        %% Getter functions
        % Get trajectory distribution from Gaussian mixture regression
        function traj_gmm = get_gmr_trajectory(obj)
            traj_gmm = obj.traj_ref;
        end

        % Get trajectory distribution from KMP model
        function traj_kmp = get_kmp_trajectory(obj)
            traj_kmp = obj.traj_kmp;
        end

        function model = get_gmm_model(obj)
            model = obj.model;
        end

        function [model_mean, model_cov, model_var] = get_prob_model(...
                obj, traj)
            % Extract mean/covariance of the model
            for i = 1:obj.n_step
                model_mean(:,i) = traj(i).mu;

                for h=1:2*obj.dim
                    model_var(h,i) = traj(i).sigma(h,h);
                    model_var(h,i) = sqrt(model_var(h,i));
                end

                model_cov(:,:,i) = traj(i).sigma;

                % Make covariance matrix symmetric
                model_cov(:,:,i) = (model_cov(:,:,i) +...
                    model_cov(:,:,i)')/2;
            end
        end

        % Sample from learned probablistic model
        function g_sample = get_samples(obj, traj, n_sample)
            % Extract mean/covariance
            [model_mean, model_cov] = obj.get_prob_model(traj);

            g_sample = cell(n_sample, 1);
            step_sample = cell(obj.n_step, 1);

            for i = 1:obj.n_step
                try
                    step_sample{i} = mvnrnd(model_mean(:,i),...
                        model_cov(:,:,i), n_sample)';
                catch
                    model_cov(:,:,i) = model_cov(:,:,i) +...
                        1e-8*eye(size(model_cov(:,:,i), 1));
                    step_sample{i} = mvnrnd(model_mean(:,i),...
                        model_cov(:,:,i), n_sample)';
                end
            end

            for i = 1:n_sample
                for j = 1:obj.n_step
                    g_sample{i}(:,:,j) = get_exp_mapping(...
                        step_sample{j}(1:6,i), obj.group_name);
                end
            end
        end

        %% Learning reference trajectory using GMM/GMR
        function learn_gmm(obj)
            % Extract pose in exponential coordinates of PCG(3)
            data_in = zeros(1+2*obj.dim, obj.n_demo*obj.n_step);
            dt = 1/obj.n_step;
            idx = 0;
            for i = 1:obj.n_demo
                for j = 1:obj.n_step
                    idx = idx+1;
                    data_in(1,idx) = j*dt;
                    data_in(2:4, idx) = obj.g_demo{i}.exponential(1:3,j);
                    data_in(5:7, idx) = obj.g_demo{i}.exponential(4:6,j);
                end

                lowIndex = (i-1)*obj.n_step+1;
                upIndex = i*obj.n_step;
                for k = 1:obj.dim
                    data_in(obj.dim+1+k,lowIndex:upIndex) = gradient(...
                        data_in(1+k,lowIndex:upIndex))/dt;
                end
            end

            % Extract the reference trajectory
            % Time step duration
            obj.model.dt = dt;

            obj.model = init_GMM_timeBased(data_in, obj.model);
            obj.model = EM_GMM(data_in, obj.model);

            % Gaussian Mixture Regression, see Eq. (17)-(19)
            [data_out, sigma_out] = GMR(obj.model,...
                (1:obj.n_step)*obj.model.dt, 1, 2:obj.model.nbVar);

            for i = 1:obj.n_step
                obj.traj_ref(i).t = i*obj.model.dt;
                obj.traj_ref(i).mu = data_out(:,i);
                obj.traj_ref(i).sigma = sigma_out(:,:,i);
            end
            
            obj.traj_ref_new = obj.traj_ref;
            obj.n_step_new = obj.n_step;
        end

        %% KMP in PCG(3)
        function compute_kmp_via_point(obj, g_via, cov_via, t_via)
            % Define the variable
            var_via = get_exp_coord(g_via, obj.group_name);
            var_via = [var_via; zeros(obj.dim,1)];

            cov_via = [cov_via, zeros(obj.dim, obj.dim);
                zeros(obj.dim, obj.dim), 1e-6*eye(obj.dim)];

            % Update the reference trajectory using desired poses
            % Insert via point to KMP model
            [obj.traj_ref_new, obj.n_step_new] = kmp_insertPoint(...
                obj.traj_ref_new, obj.n_step_new, t_via, var_via, cov_via);

            % Prediction using KMP
            [Kinv1, Kinv2] = kmp_estimateMatrix_mean_var(...
                obj.traj_ref_new, obj.n_step_new,...
                obj.kh, obj.lamda, obj.lamdac, obj.dim);

            dt = 1/obj.n_step;
            for i = 1:obj.n_step
                t = i*dt;
                [mu, sigma] = kmp_pred_mean_var(t, obj.traj_ref_new,...
                    obj.n_step_new,...
                    obj.kh, Kinv1, Kinv2, obj.lamdac, obj.dim);

                obj.traj_kmp(i).t = t;
                obj.traj_kmp(i).mu = mu;
                obj.traj_kmp(i).sigma = sigma;
            end
        end
    end
end