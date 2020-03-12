function plotFunction(time,data,title,label_x,parametrization,descriptor_type)

labelfontsize_x = 24;
labelfontsize_y = 24;
axisfontsize = 16;

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{1}{s}]$','$\omega_3[\frac{1}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{1}{s}]$','$v_3[\frac{1}{s}]$'};
            case 'screw_axis'
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{1}{s}]$','$\omega_3[\frac{1}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
        end
    case 'geometric'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\Omega_x$[rad]','$\Omega_y$[rad]','$\Omega_z$[rad]','$V_x$[m]','$V_y$[m]','$V_z$[m]'};
            case 'frenetserret'
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[m]','$V_2$[-]','$V_3$[-]'};
            case 'screw_axis'
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'dimensionless'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\Omega_x$[-]','$\Omega_y$[-]','$\Omega_z$[-]','$V_x$[-]','$V_y$[-]','$V_z$[-]'};
            case 'frenetserret'
                plotNames = {'$\Omega_1$[-]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[-]','$V_2$[-]','$V_3$[-]'};
            case 'screw_axis'
                plotNames = {'$\Omega_1$[-]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[-]','$V_2$[-]','$V_3$[-]'};
        end
end

%Plot twist
% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
plot(time,data(:,1),'Parent',subplot1,'LineWidth',2);
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');
% Create subplot
subplot2 = subplot(2,3,4,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
plot(time,data(:,4),'Parent',subplot2,'LineWidth',2,'Color','r');
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');
% Create subplot
subplot3 = subplot(2,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
plot(time,data(:,2),'Parent',subplot3,'LineWidth',2);
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');
% Create subplot
subplot4 = subplot(2,3,5,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
plot(time,data(:,5),'Parent',subplot4,'LineWidth',2,'Color','r');
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');
% Create subplot
subplot5 = subplot(2,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot5,'on');
hold(subplot5,'all');
% Create plot
plot(time,data(:,3),'Parent',subplot5,'LineWidth',2);
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');
% Create subplot
subplot6 = subplot(2,3,6,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
plot(time,data(:,6),'Parent',subplot6,'LineWidth',2,'Color','r');
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90);%,'HorizontalAlignment','right');


if ~isempty(title)
suptitle(title);
end