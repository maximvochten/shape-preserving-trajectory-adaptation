function plotFunction_struct(xvector,data1,data2,titel,label_x,parametrization,descriptor_type)


labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;
M = length(data2);

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                plotNames = {'$i_{r1}[\frac{rad}{s}]$','$i_{r2}[\frac{1}{s}]$','$i_{r3}[\frac{1}{s}]$','$i_{t1}[\frac{m}{s}]$','$i_{t2}[\frac{1}{s}]$','$i_{t3}[\frac{1}{s}]$'};
            case 'screw_axis'
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{1}{s}]$','$\omega_3[\frac{1}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
            case 'jerkaccuracy'
                    plotNames = {'$\ddot{\omega}_x[\frac{rad}{s^3}]$','$\ddot{\omega}_y[\frac{rad}{s^3}]$','$\ddot{\omega}_z[\frac{rad}{s^3}]$','$\ddot{v}_x[\frac{m}{s^3}]$','$\ddot{v}_y[\frac{m}{s^3}]$','$\ddot{v}_z[\frac{m}{s^3}]$'};
        end
    case 'geometric'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\Omega_x$[rad]','$\Omega_y$[rad]','$\Omega_z$[rad]','$V_x$[m]','$V_y$[m]','$V_z$[m]'};
            case 'frenetserret'
                plotNames = {'{$I_{r1}$[rad]}','{$I_{r2}$[-]}','{$I_{r3}$[-]}','{$I_{t1}$[m]}','{$I_{t2}$[-]}','{$I_{t3}$[-]}'};
                %plotNames = {'{\boldmath $I_{r1}[rad]$}','{\boldmath $I_{r2}[-]$}','{\boldmath $I_{r3}[-]$}','{\boldmath $I_{t1}[m]$}','{\boldmath $I_{t2}[-]$}','{\boldmath $I_{t3}[-]$}'};
            case 'screw_axis'
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'dimensionless'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\Omega_x$[-]','$\Omega_y$[-]','$\Omega_z$[-]','$V_x$[-]','$V_y$[-]','$V_z$[-]'};
            case 'frenetserret'
                plotNames = {'$I_{r1}$[-]','$I_{r2}$[-]','$I_{r3}$[-]','$I_{t1}$[-]','$I_{t2}$[-]','$I_{t3}$[-]'};
            case 'screw_axis'
                plotNames = {'$\Omega_1$[-]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[-]','$V_2$[-]','$V_3$[-]'};
        end
end

%colorspec = {[1.0 0.0 0.0];  [0.8 0.2 0.2]; [0.7 0.2 0.2]; ...
%[0.6 0.2 0.2]};

linestyles = {'-','--',':','-.','-','--',':','-.','-','--',':','-.'};

% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.6359    0.1819    0.4297    0.6806]);

% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
plot(xvector(1:end-1),data1(1:end-1,1),'-','Parent',subplot1,'LineWidth',3);
plot(0,0)
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,1),'Parent',subplot1,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
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
plot(xvector(1:end-1),data1(1:end-1,4),'-','Parent',subplot2,'LineWidth',3,'Color','b');
plot(0,0)
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,4),'Parent',subplot2,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
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
plot(xvector(1:end-1),data1(1:end-1,2),'-','Parent',subplot3,'LineWidth',3);
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,2),'Parent',subplot3,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
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
plot(xvector(1:end-1),data1(1:end-1,5),'-','Parent',subplot4,'LineWidth',3,'Color','b');
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,5),'Parent',subplot4,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
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
plot(xvector(1:end-1),data1(1:end-1,3),'-','Parent',subplot5,'LineWidth',3);
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,3),'Parent',subplot5,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
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
plot(xvector(1:end-1),data1(1:end-1,6),'-','Parent',subplot6,'LineWidth',3,'Color','b');
for j=1:M
    plot(xvector(1:end-1),data2{j}.invariants(:,6),'Parent',subplot6,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

if ~isempty(titel)
    suptitle(titel);
end
