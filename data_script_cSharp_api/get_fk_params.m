function [ee_fk_tf, ee_fk_pos, ee_fk_rotm, ee_fk_quat, ee_fk_eul] = get_fk_params(TT, f1_f0)

    % computing the tranform from robot home to robot tip using
    % the forward kinematics model by
    % multiplying all the transforms of linklengths
    % (4 x 4 x n)
    ee_fk_tf = f1_f0*TT;

    % contains x,y,z for the end-effector
    ee_fk_pos = [ee_fk_tf(1,4); ee_fk_tf(2,4); ee_fk_tf(3,4)];
    
    % obtaining the rotation matrix from the above transform
    ee_fk_rotm = ee_fk_tf(1:3,1:3);
    
    % converting the rotation matrix to quaternions
    ee_fk_quat = rotm2quat(ee_fk_tf(1:3,1:3));
    
    % converting quaternions to euler angles
    ee_fk_eul = quat2eul(ee_fk_quat);

end