function plotFunction(time,data,plotnames,title,label_x)

%Plot twist
% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
% Create subplot
subplot1 = subplot(3,2,1,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
plot(time,data(:,4),'Parent',subplot1,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{4},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot2 = subplot(3,2,2,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
plot(time,data(:,1),'Parent',subplot2,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{1},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot3 = subplot(3,2,3,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
plot(time,data(:,5),'Parent',subplot3,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{5},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot4 = subplot(3,2,4,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
plot(time,data(:,2),'Parent',subplot4,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{2},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot5 = subplot(3,2,5,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot5,'on');
hold(subplot5,'all');
% Create plot
plot(time,data(:,6),'Parent',subplot5,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{6},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot6 = subplot(3,2,6,'Parent',figure1,'YGrid','on','FontSize',12);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
plot(time,data(:,3),'Parent',subplot6,'LineWidth',2);
% Create xlabel
xlabel(label_x,'FontSize',16);
% Create ylabel
ylabel(plotnames{3},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment','right');

% %# collect axes handles
% axH = findall(gcf,'type','axes');
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% b1 = -Inf;
% b2 = -Inf;
% a1 = Inf;
% a2 = Inf;
% for i=1:2:length(axH)
%     limits = get(axH(i),'ylim');
%     if limits(1) < a1
%         a1 = limits(1);
%     end
%     if limits(2) > b1
%         b1 = limits(2);
%     end 
% end
% for i=2:2:length(axH)
%     limits = get(axH(i),'ylim');
%     if limits(1) < a2
%         a2 = limits(1);
%     end
%     if limits(2) > b2
%         b2 = limits(2);
%     end 
% end
% 
% set(axH(1:2:end),'ylim',[a1 b1])
% set(axH(2:2:end),'ylim',[a2 b2])

if ~isempty(title)
suptitle(title);
end