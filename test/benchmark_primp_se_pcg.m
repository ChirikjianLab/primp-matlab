% Benchmark script for PRIMP between SE(3) and PCG(3) formulations
%
%  Author
%    Sipu Ruan, 2023

close all; clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tunable parameters
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Number of benchmark trials
n_trial = 100;

% Number of time steps
n_step = 30;

% Number of sampled trajectories from distribution
n_sample = 5;

% Group name: 'SE', 'PCG'
group_name = {'SE', 'PCG'};

% Scaling of initial covariance
COV_INIT_SCALE = 1e-3;

% Scaling of via pose covariance
COV_VIA_POSE_SCALE = 1e-4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_paths();
cov_dim = 6;

argin.n_step = n_step;

argin.cov_dim = cov_dim;
argin.cov_init_scale = COV_INIT_SCALE;
argin.robot = loadrobot('frankaEmikaPanda', "Gravity", [0 0 -9.81]);

%% Workspace density using conditional PDF in SE(3)
res_direct_goal = cell(n_trial, length(group_name));
res_direct_via = cell(n_trial, length(group_name));
res_goal = cell(n_trial, length(group_name));
res_via = cell(n_trial, length(group_name));

for i = 1:n_trial
    clc;
    disp([num2str(i/n_trial * 100), '%'])

    % Simulate robot workspace trajectory distribution
    argin.group_name = 'SE';
    [g_mean, cov_t] = generate_trajectory(argin);

    % Via pose deviation from initial, in exponential coordinates
    via_pose_deviation_exp_coord = [pi*1e-2*rand(3,1); 0.2*rand(3,1)];

    % Desired via pose and covariance
    t_via = rand();

    step_via = floor(t_via*n_step);
    if step_via <= 0
        step_via = 1;
    elseif step_via >= n_step
        step_via = n_step;
    end

    g_via = g_mean.matrix(:,:,step_via) *...
        expm_SE(via_pose_deviation_exp_coord);
    cov_via = COV_VIA_POSE_SCALE * rand() * diag([4 4 4 1 1 1]);
    g_goal = g_mean.matrix(:,:,end) *...
        expm_SE(via_pose_deviation_exp_coord);
    cov_goal = COV_VIA_POSE_SCALE * rand() * diag([4 4 4 1 1 1]);

    for j = 1:length(group_name)
        % Initiate class
        param.n_sample = n_sample;
        param.group_name = group_name{j};

        primp_obj = PRIMP(g_mean.matrix, cov_t, param);

        res_goal{i,j}.group_name = param.group_name;
        res_via{i,j}.group_name = param.group_name;

        t_start = tic;

        % Condition on goal pose
        primp_obj.get_condition_pdf(1.0, g_goal, cov_goal);
        g_samples_goal = primp_obj.get_samples();

        % Condition on a via pose
        primp_obj.get_condition_pdf(t_via, g_via, cov_via);
        g_samples_via = primp_obj.get_samples();

        t(i,j) = toc(t_start);

        %% Distance to desired pose and original trajectory
        % Convert to group structure
        res_goal{i,j} =...
            generate_pose_struct(g_samples_goal, param.group_name);
        res_via{i,j} =...
            generate_pose_struct(g_samples_via, param.group_name);

        % Distance to initial trajectory
        traj_init{1}.matrix = g_mean.matrix;
        traj_init{1}.cov = cov_t;

        d_mean_traj.goal(i,:,j) =...
            evaluate_traj_distribution(res_goal{i,j}, traj_init);
        d_mean_traj.via(i,:,j) =...
            evaluate_traj_distribution(res_via{i,j}, traj_init);

        % Distance to desired pose
        d_desired_pose.goal(i,:,j) =...
            evaluate_desired_pose(res_goal{i,j}, g_goal, n_step);
        d_desired_pose.via(i,:,j) =...
            evaluate_desired_pose(res_via{i,j}, g_via, step_via);
    end

end

%% Evaluation of benchmarks
% Distance to initial trajectory

for j = 1:length(group_name)
    disp('===============================================================')
    disp(['Group: ', group_name{j}])

    disp('>>>> Condition on goal <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_mean_traj.goal(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_desired_pose.goal(:,:,j), 1) ))

    disp('---------------------------------------------------------------')

    disp('>>>> Condition on goal and a via pose <<<<')
    disp('---- Distance to demo (rot, tran):')
    disp(num2str( mean(d_mean_traj.via(:,:,j), 1) ))

    disp('---- Distance to desired pose (rot, tran):')
    disp(num2str( mean(d_desired_pose.via(:,:,j), 1) ))
end

% Computational time
figure; hold on;

disp('>>>> Computation time <<<<')
disp(num2str( mean(t,1) ))
boxplot(t)
ylim([0,max(max(t))])