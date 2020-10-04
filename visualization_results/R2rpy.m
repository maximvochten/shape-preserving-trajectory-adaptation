function x = R2rpy(R)
% Transform rotation matrix to roll-pitch-yaw angles
% 
% Input: x = [roll pitch yaw] angles in [rad]
% Output: R = 3x3 rotation matrix

alpha = atan2(R(2,1),R(1,1));
gamma = atan2(R(3,2),R(3,3));
beta = atan2(-R(3,1),cos(alpha)*R(1,1)+sin(alpha)*R(2,1));

x = [gamma,beta,alpha];

