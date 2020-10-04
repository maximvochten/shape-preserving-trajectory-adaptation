
function plot_twist_markers(measured_marker_coordinates,optim_result,descriptor)

figure; hold on; axis equal; view(-40,21); grid on; box on;

[N,M] = size(measured_marker_coordinates);

for j=1:M/3
    plot3(measured_marker_coordinates(:,3*j-2),measured_marker_coordinates(:,3*j-1),measured_marker_coordinates(:,3*j))
end

for j=1:9/3
    
    if strcmp(descriptor,'pose_twist')
        plot3(optim_result.marker_positions(:,3*j-2),optim_result.marker_positions(:,3*j-1),optim_result.marker_positions(:,3*j),'r')
        plot3(optim_result.marker_positions(1,3*j-2),optim_result.marker_positions(1,3*j-1),optim_result.marker_positions(1,3*j),'ro','MarkerSize',10)
        plot3(optim_result.marker_positions(end,3*j-2),optim_result.marker_positions(end,3*j-1),optim_result.marker_positions(end,3*j),'rx','MarkerSize',10)
        
        
        %                 %plot3(ISA_p(:,1),ISA_p(:,2),ISA_p(:,3),'k')
        %                 lengthisa = 1;
        %                 for k=1:(a-i)
        %                     ISA_x = screw_twist_optim(k,1:3)/norm(screw_twist_optim(k,1:3))';
        %                     line([ISA_p(k,1)-lengthisa*ISA_x(1) ISA_p(k,1)+lengthisa*ISA_x(1)],[ISA_p(k,2)-lengthisa*ISA_x(2) ISA_p(k,2)+lengthisa*ISA_x(2)],[ISA_p(k,3)-lengthisa*ISA_x(3) ISA_p(k,3)+lengthisa*ISA_x(3)])
        %                 end
        %                 plot3(ISA_p(:,1),ISA_p(:,2),ISA_p(:,3),'k')
        %                 plot3(ISA_p(1,1),ISA_p(1,2),ISA_p(1,3),'ko','MarkerSize',10)
        %plot3(ISA_p(end,1),ISA_p(end,2),ISA_p(end,3),'kx','MarkerSize',10)
    end
    
    
end

if strcmp(descriptor,'pose_twist')
    mean_marker_position =  calculate_centerMotion_plus_derivatives(optim_result.marker_positions,zeros(size(measured_marker_coordinates)),zeros(size(measured_marker_coordinates)));
else
    mean_marker_position =  calculate_centerMotion_plus_derivatives(measured_marker_coordinates,zeros(size(measured_marker_coordinates)),zeros(size(measured_marker_coordinates)));
end


plot3(mean_marker_position(:,1),mean_marker_position(:,2),mean_marker_position(:,3),'k.','LineWidth',1.5);

len=0.05;
t=1;
linewidth = '0.5';
for i=round(linspace(1,length(measured_marker_coordinates)-1,100))
    
    plot3(mean_marker_position(i,1),mean_marker_position(i,2),mean_marker_position(i,3),'k.','MarkerSize',20);
    
    if strcmp(descriptor,'pose_twist')
        
        velvec = optim_result.twist(i,4:6)/norm(optim_result.twist(i,4:6));
        omegavec =  optim_result.twist(i,1:3)/norm(optim_result.twist(i,1:3));
        
        arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*velvec,['_y' linewidth],t,2*t)
        arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*omegavec,['_m' linewidth],t,2*t)
        
        % line([mean_marker_position(i,1) mean_marker_position(i,1)+len*velvec(1)],[mean_marker_position(i,2) mean_marker_position(i,2)+len*velvec(2)],[mean_marker_position(i,3) mean_marker_position(i,3)+len*velvec(3)])
        % line(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*omegavec)
        
        
    elseif strcmp(descriptor,'frenetserret')
        
        arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*optim_result.FS_frames(:,1,i)',['_r' linewidth],t,2*t) %red = x, green = y, blue = z
        arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*optim_result.FS_frames(:,2,i)',['_g' linewidth],t,2*t)
        arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*optim_result.FS_frames(:,3,i)',['_b' linewidth],t,2*t)
        
        %arrow3(mean_marker_position(i,1:3),mean_marker_position(i,1:3)+len*optim_result.Euler_frames(:,1,i)',['_m' linewidth],t,2*t)
        
    end
    
end

axis equal;
if strcmp(descriptor,'screw_axis')
    
    
    for k=1:N-1
        p_perp(k,:) = cross(optim_result.twist(k,1:3),optim_result.twist(k,4:6))/norm(optim_result.twist(k,1:3))^2;
    end
    ISA_p = p_perp;
    
    lengthisa = 1;
    for k=1:N-1
        ISA_x = optim_result.twist(k,1:3)/norm(optim_result.twist(k,1:3))';
        line([ISA_p(k,1)-lengthisa*ISA_x(1) ISA_p(k,1)+lengthisa*ISA_x(1)],[ISA_p(k,2)-lengthisa*ISA_x(2) ISA_p(k,2)+lengthisa*ISA_x(2)],[ISA_p(k,3)-lengthisa*ISA_x(3) ISA_p(k,3)+lengthisa*ISA_x(3)])
        %arrow3(ISA_p(k,1:3),ISA_p(k,1:3)+lengthisa*ISA_x',['_r' linewidth],t,2*t) %red = x, green = y, blue = z
    end
    plot3(ISA_p(:,1),ISA_p(:,2),ISA_p(:,3),'k')
    plot3(ISA_p(1,1),ISA_p(1,2),ISA_p(1,3),'ko','MarkerSize',10)
    plot3(ISA_p(end,1),ISA_p(end,2),ISA_p(end,3),'kx','MarkerSize',10)
    axis equal;
    title('screw axis location')
end

% if strcmp(descriptor,'screw_axis')
%     figure;  plot3(p_perp(:,1),p_perp(:,2),p_perp(:,3))
% end

