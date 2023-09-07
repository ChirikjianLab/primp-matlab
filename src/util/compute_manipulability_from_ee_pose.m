function measure = compute_manipulability_from_ee_pose(pose, robot, ee_name)
% compute_manipulability_from_ee_pose Compute manipulability measure from
% end effector pose
%
% Input
%   pose   : End effector pose (4x4xn), n is the number of poses
%   robot  : Robot model used in Robotics Toolbox
%   ee_name: Name of end effector
%
% Output
%   measure: Computed manipulability measure ([rotation, translation])
%
% Author
%   Sipu Ruan, 2023
%
% Dependency
%   Robotics System Toolbox

n_step = size(pose, 3);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.2, 0.2, 0.2, 1, 1, 1];
q_init = robot.homeConfiguration;

measure = zeros(n_step, 3);
for i = 1:n_step
    q = ik(ee_name, pose(:,:,i), weights, q_init);
    J = robot.geometricJacobian(q, ee_name);

    % Compute Yoshikawa manipulability measure (including split rotation
    % and translation parts)
    M = J * J';
    m_rot = max(0, det(M(1:3,1:3)));
    m_trans = max(0, det(M(4:6,4:6)));
    measure(i,:) = [sqrt(m_rot), sqrt(m_trans), sqrt( max(0, det(M)) )];

    % Start from prior solution
    q_init = q;
end