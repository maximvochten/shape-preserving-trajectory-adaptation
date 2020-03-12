function R_all = quat2rot( q_all )
%input: rotmatrix R(3,3) or rotmatrix R(3,3,N)
%output: xyz rpy

N = size(q_all,1);
R_all = zeros(3,3,N);

for i=1:N
    
    q = q_all(i,:);
    s = q(4); % notation [vector - scalar]
    x = q(1);
    y = q(2);
    z = q(3);

    R_all(:,:,i) = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
        2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
        2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];  
end

