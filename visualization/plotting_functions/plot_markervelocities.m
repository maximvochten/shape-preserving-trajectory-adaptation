function plot_markervelocities(P,twist,ISA_frames,invariants,dt)

[N,M] = size(P);
N2 = length(invariants);

marker_vel_kalman = zeros(N,M);

% For each sample
for i=1:N  
    %pdot = omega x p + v 
     markervel = skew(twist(i,1:3))*reshape(P(i,:),3,[]) + repmat(twist(i,4:6)',1,M/3);
     marker_vel_kalman(i,:) = reshape(markervel,1,[]);
end

marker_vel_sai = zeros(N,M);
marker_vel_sai_omega = zeros(N,M);
marker_vel_sai_v = zeros(N,M);

% For each sample
for i=1:N2  
    %pdot = omega x p + v 
%      twist_ref = [ ISA_frames(1:3,1:3,i) * [invariants(i,1);0;0] ; cross(ISA_frames(1:3,4,i),ISA_frames(1:3,1:3,i)*[invariants(i,1);0;0]) + ISA_frames(1:3,1:3,i)*[invariants(i,4);0;0] ]; 
%      twist_skew = skew_T(twist_ref);

     marker_vel_sai_omega(i,:) = reshape( skew(ISA_frames(1:3,1:3,i)*[invariants(i,1);0;0]) * ( reshape(P(i,:),3,[]) - repmat(ISA_frames(1:3,4,i),1,M/3) ) ,1,[]) ;
     marker_vel_sai_v(i,:) = reshape(ISA_frames(1:3,1:3,i)*[invariants(i,4);0;0]*ones(1,M/3),1,[]);

     marker_vel_sai(i,:) = marker_vel_sai_v(i,:) + marker_vel_sai_omega(i,:);
end

N1 = N; N2 = N;

labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;

xvector = linspace(0,dt*N,N);
label_x = '$t$[s]';
plotNames = {'$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$', '$v[\frac{m}{s}]$'};

%colorspec = {[1.0 0.0 0.0];  [0.8 0.2 0.2]; [0.7 0.2 0.2]; ...
%[0.6 0.2 0.2]};

linestyles = {'-','--',':','-.'};

% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.6359    0.1819    0.4297    0.6806]);

% Create subplot
subplot1 = subplot(1,4,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot

plot(0,0)
for j=1:M/3
plot(xvector(1:N1),marker_vel_kalman(:,3*j-2),'-','Parent',subplot1,'LineWidth',2,'Color',[0.0 0.0 1.0],'LineStyle','-');
plot(xvector(1:N1),marker_vel_sai_v(:,3*j-2),'-','Parent',subplot1,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle','--');
plot(xvector(1:N2),marker_vel_sai_omega(:,3*j-2),'Parent',subplot1,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle',':');
plot(xvector(1:N2),marker_vel_sai(:,3*j-2),'Parent',subplot1,'LineWidth',2,'Color',[1.0 0.0 0.0],'LineStyle','-');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot2 = subplot(1,4,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot

for j=1:M/3
plot(xvector(1:N1),marker_vel_kalman(:,3*j-1),'-','Parent',subplot2,'LineWidth',2,'Color',[0.0 0.0 1.0],'LineStyle','-');
plot(xvector(1:N1),marker_vel_sai_v(:,3*j-1),'-','Parent',subplot2,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle','--');
plot(xvector(1:N2),marker_vel_sai_omega(:,3*j-1),'Parent',subplot2,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle',':');
plot(xvector(1:N2),marker_vel_sai(:,3*j-1),'Parent',subplot2,'LineWidth',2,'Color',[1.0 0.0 0.0],'LineStyle','-');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot3 = subplot(1,4,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
for j=1:M/3
plot(xvector(1:N1),marker_vel_kalman(:,3*j),'-','Parent',subplot3,'LineWidth',2,'Color',[0.0 0.0 1.0],'LineStyle','-');
plot(xvector(1:N1),marker_vel_sai_v(:,3*j),'-','Parent',subplot3,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle','--');
plot(xvector(1:N2),marker_vel_sai_omega(:,3*j),'Parent',subplot3,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle',':');
plot(xvector(1:N2),marker_vel_sai(:,3*j),'Parent',subplot3,'LineWidth',2,'Color',[1.0 0.0 0.0],'LineStyle','-');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(1,4,4,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
for j=1:M/3
plot(xvector(1:N1),sqrt(sum(marker_vel_kalman(:,3*j-2:3*j).^2,2)),'-','Parent',subplot4,'LineWidth',2,'Color',[0.0 0.0 1.0],'LineStyle','-');
plot(xvector(1:N1),sqrt(sum(marker_vel_sai_v(:,3*j-2:3*j).^2,2)),'-','Parent',subplot4,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle','--');
plot(xvector(1:N2),sqrt(sum(marker_vel_sai_omega(:,3*j-2:3*j).^2,2)),'Parent',subplot4,'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle',':');
plot(xvector(1:N2),sqrt(sum(marker_vel_sai(:,3*j-2:3*j).^2,2)),'Parent',subplot4,'LineWidth',2,'Color',[1.0 0.0 0.0],'LineStyle','-');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');
legend('kalman','v0','omega','total')

figure; hold on; box on;
for j=1:M/3
plot(xvector(1:N1),sqrt(sum(marker_vel_kalman(:,3*j-2:3*j).^2,2)),'-','LineWidth',2,'Color',[0.0 0.0 1.0],'LineStyle','-');
plot(xvector(1:N1),sqrt(sum(marker_vel_sai_v(:,3*j-2:3*j).^2,2)),'-','LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle','--');
plot(xvector(1:N2),sqrt(sum(marker_vel_sai_omega(:,3*j-2:3*j).^2,2)),'LineWidth',1,'Color',[1.0 0.0 0.0],'LineStyle',':');
plot(xvector(1:N2),sqrt(sum(marker_vel_sai(:,3*j-2:3*j).^2,2)),'LineWidth',2,'Color',[1.0 0.0 0.0],'LineStyle','-');
end
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');
legend('kalman','v0','omega','total')
