function [g_struct, Data, s] = parse_demo_trajectory(filenames, argin)

t_frame = 0:1/(argin.n_step-1):1;
n_demo = length(filenames);

g_demo_matrix = cell(n_demo, 1);
for i = 1:n_demo
    str = fileread(strcat(argin.data_folder, filenames(i).name));
    file = jsondecode(str);

    % Reshape the 3D matrix
    g_traj = permute(file.trajectory, [2,3,1]);


    if ~isfield(argin, {'align_method'})
        argin.align_method = "gora";
    end

    if argin.align_method == "gora"
        % Using GORA to align trajectories into unified timescale
        gora_obj = gora(g_traj, argin.n_step);
        gora_obj.run();
        g_demo_matrix{i} = gora_obj.g_opt;

    elseif argin.align_method == "interp"
        % Interpolate SE(3) trajectory
        n_step_init = file.num_step;
        t_init = 0:1/(n_step_init-1):1;
        g_demo_matrix{i} = interp_SE3_traj(t_init, g_traj, t_frame);
    end
end

% Each demo is an SE(3) trajectory
g_struct = generate_pose_struct(g_demo_matrix, argin.group_name);

% Convert to data model used in "pbdlib"
[Data, s] = convert_pose_to_pbdlib_data(g_struct);