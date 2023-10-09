function trials = load_random_trials(config_folder, n_via)

if nargin < 2
    n_via = 2;
end

for i = 1:n_via
    trials_filename = strcat(config_folder, "trials_random_via_",...
        num2str(i), ".json");

    % Load generated via point configurations
    file = jsondecode(fileread(trials_filename));

    % Reshape the 3D matrix
    trials.g_via{i} = permute(file.g_via, [2,3,1]);
    trials.cov_via{i} = permute(file.cov_via, [2,3,1]);
    trials.t_via{i} = file.t_via';
end