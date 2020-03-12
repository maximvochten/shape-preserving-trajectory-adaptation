function plot_FS_frames(position,FS_frames,titeltekst)

N = length(FS_frames);
indices = round(linspace(1,N,N));
linewidth = '0.5';
t = 3;

FS_x = squeeze(FS_frames(1:3,1,:))';
FS_y = squeeze(FS_frames(1:3,2,:))';
FS_z = squeeze(FS_frames(1:3,3,:))';

figure; hold on; axis equal; view(-40,21); grid on; box on;

plot3(position(indices,1),position(indices,2),position(indices,3),'k.--')
plot3(position(1,1),position(1,2),position(1,3),'ko','MarkerSize',10)
plot3(position(end,1),position(end,2),position(end,3),'kx','MarkerSize',10)

for i=indices
    arrow3(position(i,1:3),position(i,1:3)+0.33*FS_x(i,:),['_r' linewidth],t/3,2*t/3); %red = x, green = y, blue = z
    arrow3(position(i,1:3),position(i,1:3)+0.33*FS_y(i,:),['_g' linewidth],t/3,2*t/3);
    arrow3(position(i,1:3),position(i,1:3)+0.33*FS_z(i,:),['_b' linewidth],t/3,2*t/3);
end
axis equal;
title(titeltekst)



