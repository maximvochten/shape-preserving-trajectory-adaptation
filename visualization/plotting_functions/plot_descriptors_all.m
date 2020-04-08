function plot_descriptors_all(data1,data2,titel,h,parametrization,descriptor_type,sample_triggers)
% Can plot many types of trajectory descriptors

dt = h;
paper_experiment_type = '';
show_legend = 1;

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
            case 'pose'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$roll$[rad]','$pitch$[rad]','$yaw$[rad]','$x$[m]','$y$[m]','$z$[m]'};
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

colororders = get(gca,'Colororder');


% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.6359    0.1819    0.4297    0.6806]);

if strcmp(paper_experiment_type,'helix_varying2')
    set(gcf,'Units','normalized','OuterPosition',[0.1324    0.4389    0.7152    0.4111]);
end

% Create subplot
for i=1:6
    
    subplot1 = subplot(2,3,i,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
    box(subplot1,'on');
    hold(subplot1,'all');
    
    % Create plot demonstration
    if i==1 || i==2 || i==3
        datatoplot = unwrap(data1(:,i));
    else
        datatoplot = [data1(:,i)];
    end
    if strcmp(descriptor_type,'pose')
    plot(xvector(1:N1),datatoplot,'color',colororders(1,:),'Parent',subplot1,'LineWidth',1.5);
    else
            plot(xvector(1:N1),datatoplot,'k','Parent',subplot1,'LineWidth',2.5);
    end
    plot(0,0)
    lastsample = [];
    lastindex = [];
    % Create plot rest
    for j=1:M2
        if iscell(data2)
            data_2 = data2{j};
            %N2 = N1-size(data_2,1)+1;
            N2 = sample_triggers(j);
        end
        if strcmp(descriptor_type,'pose')
            %         if N2~=1
            %             %data_old = data2{j-1};
            %             %extra_sample = data1( sample_triggers(j) - sample_triggers(j-1) +1 , i );
            %             extra_sample = data1( N2-1 , i );
            %
            %             N2 = N2-1;
            %         else
            extra_sample = [];
            %        end
            %startindex = sample_triggers(j);
            % endindex =
            
            % plot(xvector(N2:end),[extra_sample;data_2(:,i)],'Parent',subplot1,'LineWidth',1.5,'Color',colororders(mod(j-1,7)+1,:),'LineStyle','-');
            if j~= M2
                N3 = sample_triggers(j+1) - sample_triggers(j)+1;
            else
                N3 = length(data_2);
            end
            
            if i==1 || i==2 || i==3
                datatoplot = unwrap([extra_sample;data_2(1:N3,i)]);
            else
                datatoplot = [extra_sample;data_2(1:N3,i)];
            end
            
            if i==1 && j==M2
                datatoplot = datatoplot - 2*pi;
            end
            
            plot(xvector(N2:N2+N3-1),datatoplot,'Parent',subplot1,'LineWidth',3,'Color',colororders(mod(j-1,7)+1,:),'LineStyle','-');
            
        else
            %             %data_old = data2{j-1};
            %             %extra_sample = data1( sample_triggers(j) - sample_triggers(j-1) +1 , i );
            %             extra_sample = data1( N2-1 , i );
            %
            %             N2 = N2-1;
            %         else
            
            extra_sample = [];
            %        end
            %startindex = sample_triggers(j);
            % endindex =
            
            % plot(xvector(N2:end),[extra_sample;data_2(:,i)],'Parent',subplot1,'LineWidth',1.5,'Color',colororders(mod(j-1,7)+1,:),'LineStyle','-');
            if j~= M2
                N3 = sample_triggers(j+1) - sample_triggers(j)+1;
            else
                N3 = length(data_2);
            end
            if j == 1
                plot(xvector(N2:N2+N3-2),data_2(1:N3-1,i),'Parent',subplot1,'LineWidth',1.5,'Color',colororders(mod(j-1,7)+1,:),'LineStyle','-');
                lastsample = data_2(N3,i);
                lastindex = xvector(N2+N3-1);
            else
                data_old = data2{j-1};
                extra_sample =  data_old( sample_triggers(j) - sample_triggers(j-1) +1 , i );
                extra_sample = [];
                
                plot([lastindex xvector(N2:N2+N3-1)],[lastsample;data_2(1:N3,i)],'Parent',subplot1,'LineWidth',1.5,'Color',colororders(mod(j-1,7)+1,:),'LineStyle','-');
                lastsample = data_2(N3,i);
                lastindex = xvector(N2+N3-1);
            end
        end
    end
    % Create xlabel
    xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
    % Create ylabel
    %ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
    title(plotNames{i},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');
end

%if ~isempty(titel)
%    suptitle(titel);
%end
