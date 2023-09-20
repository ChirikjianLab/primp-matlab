function trials = generate_random_trials(g_demo, n_trial, scale, folder)
% generate_random_trials Generate random experimental trials. One trial 
% includes a goal and a via pose with uncertainty.
%
%  Input
%    g_demo : Structure of pose info
%    n_trial: Number of trials
%    dataset: Name of dataset
%    folder : Folder that stores the generated trials
%
%  Output
%    trials : Structure that stores the trials
%
%  Author
%    Sipu Ruan, 2022

% Generate random poses for n_trial
for i = 1:n_trial
    % Goal pose
    trials.t_via{1}(i) = 1.0;
    [trials.g_via{1}(:,:,i), trials.cov_via{1}(:,:,i),...
        trials.step_via{1}(i)] = generate_random_pose(g_demo,...
        trials.t_via{1}(i), scale);

    % Via pose at random time step
    trials.t_via{2}(i) = rand();
    [trials.g_via{2}(:,:,i), trials.cov_via{2}(:,:,i),...
        trials.step_via{2}(i)] = generate_random_pose(g_demo,...
        trials.t_via{2}(i), scale);

end

disp("Generated random configurations!")

% Store poses
if nargin > 3
    % Goal poses
    trials_goal.g_goal = permute(trials.g_via{1}, [3,1,2]);
    trials_goal.cov_goal = permute(trials.cov_via{1}, [3,1,2]);

    json_data = jsonencode(trials_goal);
    fid = fopen(strcat(folder, 'trials_random_goal.json'), 'w');
    fprintf(fid, '%s', json_data);
    fclose(fid);

    % Via poses
    trials_via.t_via = trials.t_via{2};
    trials_via.g_via = permute(trials.g_via{2}, [3,1,2]);
    trials_via.cov_via = permute(trials.cov_via{2}, [3,1,2]);

    json_data = jsonencode(trials_via);
    fid = fopen(strcat(folder, 'trials_random_via.json'), 'w');
    fprintf(fid, '%s', json_data);
    fclose(fid);

    disp("Stored random configurations!")
end
end