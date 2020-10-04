function plot_ISA_frames(optim_result_ISA_frames,titeltekst,axislims)
% Plot ISA frames at each time step

N = length(optim_result_ISA_frames);
indices = round(linspace(1,N,N/5));
linewidth = '0.5';
t = 3;

ISA_p = squeeze(optim_result_ISA_frames(1:3,4,:))';
ISA_x = squeeze(optim_result_ISA_frames(1:3,1,:))';
ISA_y = squeeze(optim_result_ISA_frames(1:3,2,:))';
ISA_z = squeeze(optim_result_ISA_frames(1:3,3,:))';

if ~isempty(axislims)
    axis(axislims)
end
hold on; axis equal; view(-40,-50); grid on; box on;

line_ext = 1;
for i=indices
    line([ISA_p(i,1)-line_ext*ISA_x(i,1) ISA_p(i,1)+line_ext*ISA_x(i,1)],[ISA_p(i,2)-line_ext*ISA_x(i,2) ISA_p(i,2)+line_ext*ISA_x(i,2)],[ISA_p(i,3)-line_ext*ISA_x(i,3) ISA_p(i,3)+line_ext*ISA_x(i,3)],'linewidth',2)
    plot3(ISA_p(i,1)+line_ext*ISA_x(i,1),ISA_p(i,2)+line_ext*ISA_x(i,2),ISA_p(i,3)+line_ext*ISA_x(i,3),'b^','MarkerSize',5)
    
    arrow3(ISA_p(i,:),ISA_p(i,:)+0.2*ISA_x(i,:),['_r' linewidth],t/3,2*t/3);
    arrow3(ISA_p(i,:),ISA_p(i,:)+0.2*ISA_y(i,:),['_g' linewidth],t/3,2*t/3);
    arrow3(ISA_p(i,:),ISA_p(i,:)+0.2*ISA_z(i,:),['_b' linewidth],t/3,2*t/3);
end
plot3(ISA_p(indices,1),ISA_p(indices,2),ISA_p(indices,3),'k.--')
plot3(ISA_p(1,1),ISA_p(1,2),ISA_p(1,3),'ko','MarkerSize',10)
plot3(ISA_p(end,1),ISA_p(end,2),ISA_p(end,3),'kx','MarkerSize',10)
xlabel('x')
ylabel('y')
zlabel('z')
%set(gca,'CameraUpVector',[0 1 0])        % Set Y axis as vertical
%set(gca,'XDir','reverse')                % Reverse the x axis
%camroll(90)
%camup([0 0 0])
%axis equal;
axis(axislims)
title(titeltekst)



