function draw_cubes(p,R,cube_lengths,nb_rigidbodies,robot)
% Draw cubes on active figure

N1 = size(R,3);
lencx = cube_lengths(1);
lency = cube_lengths(2);
lencz = cube_lengths(3);

for j=round(linspace(1,N1,nb_rigidbodies))
    fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    vm = repmat(p(j,:),8,1) + [ zeros(3,1) ...
        R(1:3,1:3,j)*[lencx; 0; 0] R(1:3,1:3,j)*[lencx; lency; 0] ...
        R(1:3,1:3,j)*[0; lency; 0] R(1:3,1:3,j)*[0; 0; lencz] ...
        R(1:3,1:3,j)*[lencx; 0; lencz] R(1:3,1:3,j)*[lencx; lency; lencz] ...
        R(1:3,1:3,j)*[0; lency; lencz] ]';
    if robot
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',1.00);
    else
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.10,'EdgeColor',[0.90 0.15 0.15],'LineWidth',1.00);
    end
end