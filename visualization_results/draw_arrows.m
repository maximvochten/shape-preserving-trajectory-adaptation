function draw_arrows(p_obj_demo,R_demo,len,linewidth,t,nb_rigidbodies)

% Draw arrows
Rx_demo = []; 
Ry_demo = []; 
Rz_demo = []; 
p_demo = [];
N1 = size(R_demo,3);

for j=round(linspace(1,N1,nb_rigidbodies))
    Rx_demo = [Rx_demo ; R_demo(1:3,1,j)'];
    Ry_demo = [Ry_demo ; R_demo(1:3,2,j)'];
    Rz_demo = [Rz_demo ; R_demo(1:3,3,j)'];
    p_demo = [p_demo ; p_obj_demo(j,:)];
end
arrow3(p_demo,p_demo+len*Rx_demo,['_r' linewidth],t,2*t)    %_y
arrow3(p_demo,p_demo+len*Ry_demo,['_e' linewidth],t,2*t)    %_m
arrow3(p_demo,p_demo+len*Rz_demo,['_b' linewidth],t,2*t)    %_c


