function plotFunction3(time,data1,data2,title,label_x,parametrization,descriptor_type)

labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'twist'
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{1}{s}]$','$\omega_3[\frac{1}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{1}{s}]$','$v_3[\frac{1}{s}]$'};
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

% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
% Create subplot
subplot1 = subplot(1,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
plot(time,data1(:,1),'Parent',subplot1,'LineWidth',2);
if ~isempty(data2)
    plot(time(1:end-1),data2(:,1),'Parent',subplot1,'LineWidth',2,'Color','r');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0);%,'HorizontalAlignment','right');
% Create subplot
subplot2 = subplot(1,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
plot(time,data1(:,2),'Parent',subplot2,'LineWidth',2,'Color','b');
if ~isempty(data2)
    plot(time(1:end-1),data2(:,2),'Parent',subplot2,'LineWidth',2,'Color','r');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0);%,'HorizontalAlignment','right');
% Create subplot
subplot3 = subplot(1,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
plot(time,data1(:,3),'Parent',subplot3,'LineWidth',2);
if ~isempty(data2)
    plot(time(1:end-1),data2(:,3),'Parent',subplot3,'LineWidth',2,'Color','r');
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x);
% Create ylabel
ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0);%,'HorizontalAlignment','right');


if ~isempty(title)
    suptitle(title);
end