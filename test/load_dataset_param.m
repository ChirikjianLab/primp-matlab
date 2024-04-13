% Function to load demonstration dataset
%
%  Two datasets:
%    - Kinesthetic demonstrations using Panda arm (6D pose)
%    - LASA handwriting dataset (2D position)
%
%  Author
%    Sipu Ruan, 2022

function demo_type = load_dataset_param(dataset_name, id)

% Type of demonstration
switch dataset_name
    case 'panda_arm'
        demo_type = {'simulation/circle', 'simulation/letter_N',...
            'simulation/letter_U', 'simulation/letter_S',...
            'real/trajectory/pouring/default', 'real/trajectory/scooping/default',...
            'real/trajectory/transporting/default',...
            'real/trajectory/opening/sliding', 'real/trajectory/opening/rotating_down',...
            'real/trajectory/opening/rotating_left'};
    case 'lasa_handwriting/pose_data'
        demo_type = {'Angle', 'BendedLine', 'CShape',...
            'DoubleBendedLine', 'GShape', 'heee', 'JShape', 'JShape_2',...
            'Khamesh', 'Leaf_1', 'Leaf_2', 'Line', 'LShape', 'NShape',...
            'PShape', 'RShape', 'Saeghe', 'Sharpc', 'Sine', 'Snake',...
            'Spoon', 'Sshape', 'Trapezoid', 'Worm', 'WShape', 'Zshape'};
end

if nargin > 1
    demo_type = demo_type(id);
end