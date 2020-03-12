function obj_frames_theta = interpR(theta,obj_frames_t,theta_n)
% interpR is like interp1 but for rotation matrices
% interpR assume a constant rotational velocity vector between successive samples

j = 1;
for i=1:length(theta_n)
    
    while theta_n(i) > theta(j+1)
        j=j+1;
    end
    
    theta0 = theta(j);
    theta1 = theta(j+1);
    R0 = obj_frames_t(:,:,j);
    R1 = obj_frames_t(:,:,j+1);
    
    R = R0 * expm( (theta_n(i)-theta0)/(theta1-theta0) * logm(R0'*R1) );
    
    obj_frames_theta(:,:,i) = R;
    
end