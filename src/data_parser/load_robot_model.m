function [robot_model_name, ee_name, num_link] = load_robot_model(robot_name)
num_link = 6;

switch robot_name
    case 'panda_arm'
        robot_model_name = 'frankaEmikaPanda';
        ee_name = 'panda_link8';
        num_link = 7;

    case 'ur5'
        robot_model_name = 'universalUR5';
        ee_name = 'ee_link';

    case 'kinovaGen3'
        robot_model_name = 'kinovaGen3';
        ee_name = 'EndEffector_Link';
        num_link = 7;

    case 'kukaIiwa7'
        robot_model_name = 'kukaIiwa7';
        ee_name = 'iiwa_link_ee';
        num_link = 7;

    case 'abbYumi_right_arm'
        robot_model_name = 'abbYumi';
        ee_name = 'yumi_link_7_r';

    case 'atlas_left_hand'
        robot_model_name = 'atlas';
        ee_name = 'l_hand_force_torque';

    case 'rethinkBaxter_right_hand'
        robot_model_name = 'rethinkBaxter';
        ee_name = 'right_hand';

    case 'robotisOpenManipulator'
        robot_model_name = 'robotisOpenManipulator';
        ee_name = 'end_effector_link';
end