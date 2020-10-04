function R = rpy2R(x)
% Transform roll pitch yaw angles to a rotation matrix
% 
% Input: x = [roll pitch yaw] angles in [rad]
% Output: R = 3x3 rotation matrix

R = rotz(x(3)) * roty(x(2)) * rotx(x(1));

 end

function R = rotx(phi)
R = [1        0          0; ...
     0        cos(phi)   -sin(phi); ...
     0        sin(phi)   cos(phi)];
end

function R = roty(beta)
R = [cos(beta)  0 sin(beta); ...
     0          1 0; ...
     -sin(beta) 0 cos(beta)];
end

function R = rotz(alpha)
R = [cos(alpha)  -sin(alpha) 0; ...
     sin(alpha)  cos(alpha)  0; ...
     0           0           1];
end