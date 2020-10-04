function plot_invariants_model_online(time_meas,model_invariants,invariants_online,invariants_meas,predicted_invariants,title,save_frames,i)

online_samples = size(invariants_online,1);

h=figure(12);
movegui(h,'north')
subplot(3,2,1)
hold on
plot(time_meas,model_invariants(:,1),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,1),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,1),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,1),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('\omega_1[rad/s]')
subplot(3,2,3)
hold on
plot(time_meas,model_invariants(:,2),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,2),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,2),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,2),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('\omega_2[1/s]')
subplot(3,2,5)
hold on
plot(time_meas,model_invariants(:,3),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,3),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,3),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,3),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('\omega_3[1/s]')
subplot(3,2,2)
hold on
plot(time_meas,model_invariants(:,4),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,4),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,4),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,4),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('v_1[mm/s]')
subplot(3,2,4)
hold on
plot(time_meas,model_invariants(:,5),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,5),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,5),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,5),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('v_2[1/s]')
subplot(3,2,6)
hold on
plot(time_meas,model_invariants(:,6),'g','linewidth',1);
%plot(time_meas,invariants_meas(:,6),'b','linewidth',1);
plot(time_meas(1:online_samples),invariants_online(:,6),'r','linewidth',1);
plot(time_meas,predicted_invariants(:,6),'k','linewidth',2);
yL = get(gca,'YLim');
line([time_meas(online_samples) time_meas(online_samples)],yL,'Color','r');
ylabel('v_3[1/s]')
suptitle(title)

if(save_frames && mod(i,2)==0)
   save2pdf(['figures/plot_scheppeneten_invariants_' num2str(i/2-1) '.pdf'],h)
end