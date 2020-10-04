function plotFunctionCompare7(data1,data2,plotnames,title)

%Plot twist
% Create figure
figure1 = figure('Name','Figure','Color',[1 1 1]);
% Create subplot
subplot1 = subplot(3,3,1,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
plot(data1(:,1)./data1(:,1),'b','Parent',subplot1,'LineWidth',2);
plot(data2(:,1)./data2(:,1),'r','Parent',subplot1,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{1},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot2 = subplot(3,3,2,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
plot(data1(:,4),'b','Parent',subplot2,'LineWidth',2);
plot(data2(:,4),'r','Parent',subplot2,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{4},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot3 = subplot(3,3,4,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
plot(data1(:,2),'b','Parent',subplot3,'LineWidth',2);
plot(data2(:,2),'r','Parent',subplot3,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{2},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot4 = subplot(3,3,5,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
plot(data1(:,5),'b','Parent',subplot4,'LineWidth',2);
plot(data2(:,5),'r','Parent',subplot4,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{5},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot5 = subplot(3,3,7,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot5,'on');
hold(subplot5,'all');
% Create plot
plot(data1(:,3),'b','Parent',subplot5,'LineWidth',2);
plot(data2(:,3),'r','Parent',subplot5,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{3},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot6 = subplot(3,3,8,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
plot(data1(:,6),'b','Parent',subplot6,'LineWidth',2);
plot(data2(:,6),'r','Parent',subplot6,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{6},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');
% Create subplot
subplot7 = subplot(3,3,3,'Parent',figure1,'YGrid','on','FontSize',14);
box(subplot7,'on');
hold(subplot7,'all');
% Create plot
plot(data1(:,1),'b','Parent',subplot7,'LineWidth',2);
plot(data2(:,1),'r','Parent',subplot7,'LineWidth',2);
% Create xlabel
xlabel('samples','FontSize',14);
% Create ylabel
ylabel(plotnames{7},'Interpreter','LaTex','FontSize',24,'Rotation',0,'HorizontalAlignment','right');

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

h=suptitle(title);
set(h,'FontSize',20)