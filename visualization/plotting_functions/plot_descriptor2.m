function plot_descriptor2(data1,data2,plot_parameters)

descriptor_type = plot_parameters.descriptor;
parametrization = plot_parameters.parameterization;
dt = plot_parameters.h;
paper_experiment_type = plot_parameters.paper_experiment_type;
show_legend = plot_parameters.show_legend;

labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;
linewidth1 = 2;
linewidth2 = 1.5;
scale_Yaxis = 1;
markersize = 8;

if isempty(data2)
    M2 = 0;
    N2 = 0;
elseif iscell(data2)
    M2 = length(data2);
    N2 = size(data2{1},1);
else
    data_2=data2;
    N2 = size(data2,1);
    M2 = 1;
end
if isempty(data1)
    M1 = 0;
    N1 = 0;
else
    N1 = length(data1);
    M1 = 1;
    
end
N = max(N2,N1);

% scale figures the same



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
            case 'frenetserret'
                xvector = linspace(0,dt*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{r1}[\frac{rad}{s}]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$','$i_{t1}[\frac{m}{s}]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$'};
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
set(gcf,'Units','normalized','OuterPosition',[0.6359    0.1819    0.4297    0.6806]);

if strcmp(paper_experiment_type,'helix_varying2')
    set(gcf,'Units','normalized','OuterPosition',[0.1324    0.4389    0.7152    0.4111]);
end

% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,1),'-','Parent',subplot1,'LineWidth',2);
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,1),'Parent',subplot1,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot2 = subplot(2,3,4,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,4),'-','Parent',subplot2,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,4),'Parent',subplot2,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot3 = subplot(2,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,2),'-','Parent',subplot3,'LineWidth',2);
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,2),'Parent',subplot3,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(2,3,5,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,5),'-','Parent',subplot4,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,5),'Parent',subplot4,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot5 = subplot(2,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot5,'on');
hold(subplot5,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,3),'-','Parent',subplot5,'LineWidth',2);
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,3),'Parent',subplot5,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot6 = subplot(2,3,6,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,6),'-','Parent',subplot6,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,6),'Parent',subplot6,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

%if ~isempty(titel)
%    suptitle(titel);
%end
