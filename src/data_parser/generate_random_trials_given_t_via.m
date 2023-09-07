function trials = generate_random_trials_given_t_via(g_demo, t_via,...
    scale, folder)
% generate_random_trials Generate random experimental trials. One trial
% includes a goal and a via pose with uncertainty.
%
%  Input
%    g_ref  : Structure of pose info
%    t_via  : Array (num. trials x num. via points) of requested via point
%             time step, range [0,1]
%    scale  : Scale of the mean and covariance
%    folder : Folder that stores the generated trials
%
%  Output
%    trials : Structure that stores the trials
%
%  Author
%    Sipu Ruan, 2023

% Generate random poses for n_trial
n_trial = size(t_via, 1);
n_via = size(t_via, 2);

for j = 1:n_via
    for i = 1:n_trial
        % Via-point pose
        trials.t_via{j}(i) = t_via(i, j);
        [trials.g_via{j}(:,:,i), trials.cov_via{j}(:,:,i),...
            trials.step_via{j}(i)] = generate_random_pose(g_demo,...
            trials.t_via{j}(i), scale);

    end

    % Store poses
    if nargin > 4
        trials_via.t_via = trials.t_via{j};
        trials_via.g_via = permute(trials.g_via{j}, [3,1,2]);
        trials_via.cov_via = permute(trials.cov_via{j}, [3,1,2]);

        json_data = jsonencode(trials_via);
        fid = fopen(...
            strcat(folder, 'trials_random_via_', num2str(j), '.json'), 'w');
        fprintf(fid, '%s', json_data);
        fclose(fid);
    end
end