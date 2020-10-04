function plot_twists_new( data_locations )

movementsConsidered = gather_motion_invariants(data_locations,'screwTwist','Gewoon1',200);
colors = hsv(length(movementsConsidered));



    
    for j=1:length(movementsConsidered) % bewegingen
        for i=1:6 % omega1
        % Create figure
        movement = movementsConsidered(i);
        motionName = movement.motionName;
        
    figure1 = figure('Name','d','Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    invariants_motion = movementsConsidered(j).invariants;
    invariants_motion = invariants_motion(:,i:6:end);
    
    p = plot(invariants_motion,'Parent',axes1);
    set(p,'Color',colors(j,:),'LineWidth',1)
    end
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Create ylabel
    ylabel('\Omega_1 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    title(['Model ' strrep(motionName,'_','\_') '-' 'Gewoon1' '-trial'],'FontSize',16);
    
end

end

