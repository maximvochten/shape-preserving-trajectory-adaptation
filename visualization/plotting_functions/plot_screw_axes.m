function plot_screw_axes(twist)
% Plot the screw axis from the given twist

N = length(twist);
indices = round(linspace(1,N,100));

ISA_p = zeros(N,3);
ISA_x = zeros(N,3);
for i=1:N
    ISA_p(i,:) = cross(twist(i,1:3),twist(i,4:6))/norm(twist(i,1:3))^2;
    ISA_x(i,:) = twist(i,1:3)/norm(twist(i,1:3));
end

figure; hold on; axis equal; view(-40,21); grid on; box on;
for i=indices
    if norm(twist(i,1:3))^2 > 2
        line([ISA_p(i,1)-0.1*ISA_x(i,1) ISA_p(i,1)+0.1*ISA_x(i,1)],[ISA_p(i,2)-0.1*ISA_x(i,2) ISA_p(i,2)+0.1*ISA_x(i,2)],[ISA_p(i,3)-0.1*ISA_x(i,3) ISA_p(i,3)+0.1*ISA_x(i,3)],'linewidth',1.5)
        plot3(ISA_p(i,1)+0.1*ISA_x(i,1),ISA_p(i,2)+0.1*ISA_x(i,2),ISA_p(i,3)+0.1*ISA_x(i,3),'b^','MarkerSize',5)
    end
end
plot3(ISA_p(indices,1),ISA_p(indices,2),ISA_p(indices,3),'k.--')
plot3(ISA_p(1,1),ISA_p(1,2),ISA_p(1,3),'ko','MarkerSize',10)
plot3(ISA_p(end,1),ISA_p(end,2),ISA_p(end,3),'kx','MarkerSize',10)
axis equal;
title('screw axis location given screw twist')