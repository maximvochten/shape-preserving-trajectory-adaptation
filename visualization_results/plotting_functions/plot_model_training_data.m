function plot_model_training_data(invariantsLocation,modelLocations,params)

nb_invariants_samples = params.nb_samples;
typeOfInvariants = params.typeOfInvariants;
executionType = params.executionType;

modelsConsidered = gather_models(modelLocations,1,typeOfInvariants{1});
invariantsConsidered = gather_motion_invariants(invariantsLocation.(typeOfInvariants{1}),typeOfInvariants{1},executionType{1},nb_invariants_samples);

colors = hsv(length(modelsConsidered));

for i = 1:length(modelsConsidered)
    model = modelsConsidered(i).models.O1.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,1:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    model = modelsConsidered(i).models.O2.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,2:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    model = modelsConsidered(i).models.O3.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,3:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    model = modelsConsidered(i).models.V1.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,4:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    model = modelsConsidered(i).models.V2.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,5:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    model = modelsConsidered(i).models.V3.model;
    X = modelsConsidered(i).models.trainingData.('Gewoon1');
    motionName = modelsConsidered(i).motionName;
    invariants_motion = invariantsConsidered(i).invariants;
    invariants_motion = invariants_motion(:,6:6:end);
    
    % Create figure
    figure1 = figure('Name',motionName,'Color',[1 1 1]);
    % Create axes
    axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
    hold(axes1,'all');
    hold on
    p = plot(model,'-','LineWidth',3);
    set(p,'Color','r')
    p = plot(invariants_motion(:,X),'-');
    set(p,'Color','b','LineWidth',1)
    % Create xlabel
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    % Createylabel
    ylabel('V_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    % Create title
    % title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
    title(['Model motion ' strrep(motionName,'_','\_')],'FontSize',16);
    
    
    %         subplot(212)
    %
    %          model = modelsConsidered(i).models.O2.model;
    %     X = modelsConsidered(i).models.trainingData;
    %     motionName = modelsConsidered(i).motionName;
    %     invariants_motion = invariantsConsidered(i).invariants;
    %     invariants_motion = invariants_motion(:,2:6:end);
    %
    %         hold on
    %     p = plot(model,'LineWidth',4);
    %     set(p,'Color',colors(i,:))
    %     p = plot(invariants_motion(:,X));
    %     set(p,'Color',colors(i,:),'LineWidth',1)
    %     % Create xlabel
    %     xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    %     % Create ylabel
    %     ylabel('\Omega_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    %     % Create title
    %     title(['Model ' strrep(motionName,'_','\_') '-' executionType '-trial'],'FontSize',16);
end

figure1 = figure('Name','All models','Color',[1 1 1]);
axes1 = axes('Parent',figure1,'YGrid','on','FontSize',14,'FontName','Georgia');
hold(axes1,'all');
for i = 1:length(modelsConsidered)
    subplot(231)
    hold on
    model = modelsConsidered(i).models.O1.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('\Omega_1 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    title('All models','FontSize',16);
    
    subplot(232)
    hold on
    model = modelsConsidered(i).models.O2.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('\Omega_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    
    subplot(233)
    hold on
    model = modelsConsidered(i).models.O3.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('\Omega_3 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    
    subplot(234)
    hold on
    model = modelsConsidered(i).models.V1.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('v_1 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    
    subplot(235)
    hold on
    model = modelsConsidered(i).models.V2.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('v_2 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    
    subplot(236)
    hold on
    model = modelsConsidered(i).models.V3.model;
    p = plot(model);
    set(p,'Color',colors(i,:),'LineWidth',4)
    xlabel('samples','FontSize',14,'HorizontalAlignment','left');
    ylabel('v_3 [-]','FontSize',14,'Rotation',0,'HorizontalAlignment','right');
    
end
