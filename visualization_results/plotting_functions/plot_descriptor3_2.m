function plot_descriptor3_2(data1,data2,plot_parameters)

descriptor = plot_parameters.descriptor;
parameterization = plot_parameters.parameterization;
dt = plot_parameters.h;
paper_experiment_type = plot_parameters.paper_experiment_type;
show_legend = plot_parameters.show_legend;
method = plot_parameters.method;


labelfontsize_x = 18;
labelfontsize_y = 22;
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



% if strcmp(paper_experiment_type,'stationary2')
%     data1(round(N/2),4:6) = NaN;
% end

switch parameterization
    case 'timebased'
        switch descriptor
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
        switch descriptor
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
        switch descriptor
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


% if strcmp(paper_experiment_type,'stationary2')
%     xvector = 1:N;
%     label_x = 'sample';
% end

if M2==1 && strcmp(method,'OCP')
    linestyles = {'-'};
    linecolors = {'r'};
elseif  M2==1 && strcmp(method,'KS')
    linestyles = {':'};
    linecolors = {'k'};
else
    linestyles = {'-','--',':','-.','-','--',':','-.','-','--',':','-.'};
    linecolors = {'r','k','r'};
end

% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.1323    0.4380    0.5172    0.4120]);

if strcmp(paper_experiment_type,'helix_varying2')
    set(gcf,'Units','normalized','OuterPosition',[ 0.1323    0.3593    0.8510    0.4907]);
end

if strcmp(paper_experiment_type,'noise')
    set(gcf,'Units','normalized','OuterPosition',[0.1323    0.4093    0.6406    0.4407]);
end

if strcmp(paper_experiment_type,'stationary2')
    set(gcf,'Units','normalized','OuterPosition',[0.1323    0.2676    0.6359    0.5824]);
end

if strcmp(paper_experiment_type,'inflection')
   set(gcf,'Units','normalized','OuterPosition',[0.1323    0.2815    0.6823    0.5685]);
    
end

% Create subplot
subplot2 = subplot(1,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,4),'-','Parent',subplot2,'LineWidth',linewidth1,'Color','b');
    for k=1:N1
        if isnan(data1(k,4))
            plot(xvector(k),0,'o','Parent',subplot2,'MarkerSize',markersize,'Color','b','LineWidth',linewidth1);
        end
    end
end
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,4),'Parent',subplot2,'LineWidth',linewidth2,'Color',linecolors{j},'LineStyle',linestyles{j});
end




plot(0,0)

% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(1,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,5),'-','Parent',subplot4,'LineWidth',linewidth1,'Color','b');
    for k=1:N1
        if isnan(data1(k,5))
            plot(xvector(k),data1(k-1,5),'o','Parent',subplot4,'MarkerSize',markersize,'Color','b','LineWidth',linewidth1);
        end
    end
end
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,5),'Parent',subplot4,'LineWidth',linewidth2,'Color',linecolors{j},'LineStyle',linestyles{j});
end




plot(0,0)

% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot6 = subplot(1,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,6),'-','Parent',subplot6,'LineWidth',linewidth1,'Color','b');
    for k=1:N1
        if isnan(data1(k,6))
            plot(xvector(k),data1(k-1,6),'o','Parent',subplot6,'MarkerSize',markersize,'Color','b','LineWidth',linewidth1);
        end
    end
end
for j=1:M2
    if iscell(data2)
        data_2 = data2{j};
    end
    plot(xvector(1:N2),data_2(:,6),'Parent',subplot6,'LineWidth',linewidth2,'Color',linecolors{j},'LineStyle',linestyles{j});
end




plot(0,0)

% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',0,'HorizontalAlignment','right');
title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% if ~isempty(titel)
%     suptitle(titel);
% end

% if scale_Yaxis
%     AxesHandles = [subplot2 subplot4 subplot6];
%     allYLim = get(AxesHandles, {'YLim'});
%     allYLim = cat(2, allYLim{:});
%     set(AxesHandles, 'YLim', [min(allYLim), max(allYLim)]);
% end

% legend(subplot2,'\sigma = 0.001 m','\sigma = 0.005 m','\sigma = 0.01 m')%,'\sigma = 0.02 m')
% legend(subplot4,'\sigma = 0.001 m','\sigma = 0.005 m','\sigma = 0.01 m')%,'\sigma = 0.02 m')
% legend(subplot6,'\sigma = 0.001 m','\sigma = 0.005 m','\sigma = 0.01 m')%,'\sigma = 0.02 m')

%legend(subplot2,'w_1 = 1e-1','w_1 = 1e-2','w_1 = 1e-3')%,'\sigma = 0.02 m')
%legend(subplot4,'w_1 = 1e-3','w_1 = 1e-4','w_1 = 1e-5')%,'\sigma = 0.02 m')
%legend(subplot6,'w_1 = 1e-3','w_1 = 1e-4','w_1 = 1e-5')%,'\sigma = 0.02 m')

if show_legend && strcmp(paper_experiment_type,'noise')
    lgd1 = legend(subplot2,'calculated invariant','ground truth invariant','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd1.Box = 'on';
    lgd2 = legend(subplot4,'calculated invariant','ground truth invariant','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd2.Box = 'on';
    lgd3 = legend(subplot6,'calculated invariant','ground truth invariant','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd3.Box = 'on';
elseif show_legend && strcmp(paper_experiment_type,'helix_varying2')
    lgd1 = legend(subplot2,'ground truth','OCP invariants','KS invariants','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd1.Box = 'on';
    lgd2 = legend(subplot4,'ground truth','OCP invariants','KS invariants','Location','NorthEast');%,'\sigma = 0.02 m')
    lgd2.Box = 'on';
    lgd3 = legend(subplot6,'ground truth','OCP invariants','KS invariants','Location','NorthEast');%,'\sigma = 0.02 m')
    lgd3.Box = 'on';
elseif show_legend && strcmp(paper_experiment_type,'stationary2') && strcmp(method,'OCP')
    lgd1 = legend(subplot2,'ground truth','OCP invariants','Location','North');%,'\sigma = 0.02 m')
    lgd1.Box = 'on';
    %set(lgd1,...
    %    'Position',[0.22808374413366 0.197767145135566 0.112469061146868 0.12350478731274]);
    lgd2 = legend(subplot4,'ground truth','OCP invariants','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd2.Box = 'on';
    set(lgd2,...
        'Position',[0.549247110470294 0.197767145135567 0.112469061146868 0.12350478731274]);
    lgd3 = legend(subplot6,'ground truth','OCP invariants','Location','SouthEast');%,'\sigma = 0.02 m')
    lgd3.Box = 'on';
elseif show_legend && strcmp(paper_experiment_type,'stationary2') && strcmp(method,'KS')
    lgd1 = legend(subplot2,'ground truth','KS invariants','Location','North');%,'\sigma = 0.02 m')
    lgd1.Box = 'on';
    lgd2 = legend(subplot4,'ground truth','KS invariants','Location','East');%,'\sigma = 0.02 m')
    lgd2.Box = 'on';
    %set(lgd2,...
     %   'Position',[ 0.536252060611256 0.695474479034947 0.12732054664947 0.123504787312741]);
    lgd3 = legend(subplot6,'ground truth','KS invariants','Location','NorthEast');%,'\sigma = 0.02 m')
    lgd3.Box = 'on';
elseif show_legend && strcmp(paper_experiment_type,'inflection')
    lgd1 = legend(subplot2,'ground truth','OCP invariants','KS invariants','OCP* invariants','Location','South');%,'\sigma = 0.02 m')
    lgd1.Box = 'on';
    lgd2 = legend(subplot4,'ground truth','OCP invariants','KS invariants','OCP* invariants','Location','SouthWest');%,'\sigma = 0.02 m')
    lgd2.Box = 'on';
    lgd3 = legend(subplot6,'ground truth','OCP invariants','KS invariants','OCP* invariants','Location','East');%,'\sigma = 0.02 m')
    lgd3.Box = 'on';
        set(lgd3,...
        'Position',[ 0.810823010795503 0.518559926972193 0.159249228626582 0.134615387558297 ]);
end
