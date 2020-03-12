function plot_descriptor3(data1,data2,titel,dt,parametrization,descriptor_type)

labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;
linewidth1 = 2;
linewidth2 = 1.5;

if isempty(data2)
    M = 0;
    N2 = 0;
else
    N2 = size(data2,1);
    M = 1;
end
if isempty(data1)
    M1 = 0;
    N1 = 0;
else
    N1 = length(data1);
    M1 = 1;
    
end
N = max(N2,N1);

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'screw_twist'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
                %             case 'frenetserret'
                %                 xvector = linspace(0,dt*N,N);
                %                 label_x = '$t$[s]';
                %                 plotNames = {'$i_{r1}[\frac{rad}{s}]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$','$i_{t1}[\frac{m}{s}]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{1}[\frac{m}{s}]$','$i_{2}[\frac{rad}{s}]$','$i_{3}[\frac{rad}{s}]$'};
            case 'screw_axis'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{rad}{s}]$','$\omega_3[\frac{rad}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
            case 'jerkaccuracy'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\ddot{\omega}_x[\frac{rad}{s^3}]$','$\ddot{\omega}_y[\frac{rad}{s^3}]$','$\ddot{\omega}_z[\frac{rad}{s^3}]$','$\ddot{v}_x[\frac{m}{s^3}]$','$\ddot{v}_y[\frac{m}{s^3}]$','$\ddot{v}_z[\frac{m}{s^3}]$'};
        end
        
    case 'geometric'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_x$[rad]','$\Omega_y$[rad]','$\Omega_z$[rad]','$V_x$[m]','$V_y$[m]','$V_z$[m]'};
            case 'screw_twist'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'{$I_{r1}$[rad]}','{$I_{r2}$[rad]}','{$I_{r3}$[rad]}','{$I_{t1}$[m]}','{$I_{t2}$[rad]}','{$I_{t3}$[rad]}'};
                %plotNames = {'{\boldmath $I_{r1}[rad]$}','{\boldmath $I_{r2}[-]$}','{\boldmath $I_{r3}[-]$}','{\boldmath $I_{t1}[m]$}','{\boldmath $I_{t2}[-]$}','{\boldmath $I_{t3}[-]$}'};
            case 'screw_axis'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'dimensionless'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_x$[-]','$\Omega_y$[-]','$\Omega_z$[-]','$V_x$[-]','$V_y$[-]','$V_z$[-]'};
            case 'screw_twist'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$I_{r1}$[-]','$I_{r2}$[-]','$I_{r3}$[-]','$I_{t1}$[-]','$I_{t2}$[-]','$I_{t3}$[-]'};
            case 'screw_axis'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[-]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[-]','$V_2$[-]','$V_3$[-]'};
        end
end

%colorspec = {[1.0 0.0 0.0];  [0.8 0.2 0.2]; [0.7 0.2 0.2]; ...
%[0.6 0.2 0.2]};

linestyles = {'-','--',':','-.'};

% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
set(gcf,'Units','normalized')%;,'OuterPosition',[1.4458    0.2787    0.7453    0.6083]);

% Create subplot
subplot2 = subplot(3,1,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
for j=1:M
    plot(xvector(1:N2),data2(:,4),'Parent',subplot2,'LineWidth',linewidth2,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
plot(0,0)
for j=1:M1
    plot(xvector(1:N1),data1(:,4),'-','Parent',subplot2,'LineWidth',linewidth1,'Color','b');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
%title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(3,1,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
for j=1:M
    plot(xvector(1:N2),data2(:,5),'Parent',subplot4,'LineWidth',linewidth2,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
plot(0,0)
for j=1:M1
    plot(xvector(1:N1),data1(:,5),'-','Parent',subplot4,'LineWidth',linewidth1,'Color','b');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
%title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot6 = subplot(3,1,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
for j=1:M
    plot(xvector(1:N2),data2(:,6),'Parent',subplot6,'LineWidth',linewidth2,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
plot(0,0)
for j=1:M1
    plot(xvector(1:N1),data1(:,6),'-','Parent',subplot6,'LineWidth',linewidth1,'Color','b');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
%title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

if ~isempty(titel)
    suptitle(titel);
end
