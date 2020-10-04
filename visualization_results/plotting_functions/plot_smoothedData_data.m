function plot_smoothedData_data( data_locations )
% plot to see how much smoothed data deviates from the original data

movementsConsidered = find_all_files_in_directory(data_locations);

for i=1:length(movementsConsidered)
    movement = movementsConsidered(i);
    smoothedData = load(movement.location);
    executionType = movement.executionType;
    motionName = movement.motionName;
    number = movement.number;
    if (strcmp(executionType,'Gewoon1') )
        [~, maxindex] = max(sum(~isnan(smoothedData.smoothedData.smoothedData.Smooth)));
        %only x-coordinate for now
        YMatrix1 = [smoothedData.smoothedData.smoothedData.Smooth(:,maxindex*3-2)  ...
                    smoothedData.smoothedData.smoothedData.Data(:,maxindex*3-2)];
        % Create figure
        figure1 = figure('Name',motionName,'Color',[1 1 1]);
        % Create axes
        axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
        hold(axes1,'all');
        % Create multiple lines using matrix input to plot
        plot1 = plot(YMatrix1,'Parent',axes1,'LineWidth',2);
        set(plot1(1),'Color',[1 0 0],'DisplayName','Smoothed');
        set(plot1(2),'Color',[0 0 1],'DisplayName','Measurement');
        % Create xlabel
        xlabel('samples','FontSize',14,'HorizontalAlignment','left');
        % Create ylabel
        ylabel('X-position [mm]','FontSize',14,'HorizontalAlignment','left');
        % Create title
        title(['Preprocessed ' strrep(motionName,'_','\_') '-' executionType '-trial' number '-' 'x'],'FontSize',16);
        % Create legend
        legend1 = legend(axes1,'show');
        set(legend1,'Position',[0.679160855416066 0.795951675903158 0.21931659693166 0.111729452054795]);
    end
end

% error = smoothedData.smoothedData.smoothedData.Data(:,3);
%
% [d11,d13,d23]=calculate_distances(smoothedData.smoothedData.smoothedData.Smooth);
% [d_11,d_13,d_23]=calculate_distances(smoothedData.smoothedData.smoothedData.Data);
% figure
% hold on
% %plot(d11)
% %plot(d13)
% plot(d23)
% %plot(d_11,'r')
% %plot(d_13,'r')
% plot(d_23,'r')
% ylabel('d^2')
% xlabel('n [samples]')
% set(get(gca,'YLabel'),'Rotation',0)
% set(get(gca,'YLabel'),'fontsize',20)
% set(get(gca,'XLabel'),'fontsize',20)
% set(gca,'FontSize',16)
% title('Distance between markers squared [m^2]')
% end
%
% function [d_12, d_13, d_23] = calculate_distances(P)
% d_12     = (P(:,4)-P(:,1)).^2+(P(:,5)-P(:,2)).^2+(P(:,6)-P(:,3)).^2;
% d_13     = (P(:,7)-P(:,1)).^2+(P(:,8)-P(:,2)).^2+(P(:,9)-P(:,3)).^2;
% d_23     = (P(:,7)-P(:,4)).^2+(P(:,8)-P(:,5)).^2+(P(:,9)-P(:,6)).^2;
% end
