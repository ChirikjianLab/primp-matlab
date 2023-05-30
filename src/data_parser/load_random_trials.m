function trials = load_random_trials(config_folder)

trials_goal_filename = strcat(config_folder, "trials_random_goal.json");
trials_via_filename = strcat(config_folder, "trials_random_via.json");

% Load generated goal configurations
goal_file = jsondecode(fileread(trials_goal_filename));

% Reshape the 3D matrix
trials.g_via{1} = permute(goal_file.g_goal, [2,3,1]);
trials.cov_via{1} = permute(goal_file.cov_goal, [2,3,1]);
trials.t_via{1} = ones(1, size(trials.g_via{1}, 3));

% Load generated via pose configurations
via_file = jsondecode(fileread(trials_via_filename));

% Reshape the 3D matrix
trials.g_via{2} = permute(via_file.g_via, [2,3,1]);
trials.cov_via{2} = permute(via_file.cov_via, [2,3,1]);
trials.t_via{2} = via_file.t_via';