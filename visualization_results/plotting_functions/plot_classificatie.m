close all
clear
clc

global params

motion = 'Tas_uitgieten'
type = 'TimeBased'

addpath(['Trials_runa/trials_measurement_2/' motion '/Gewoon1'])
addpath(['DTW_models/Models_runa/models_measurement_2/models_' type])


load([motion '.mat'])
model_o1 = MODELS1.O1.model;
% figure
% hold on
% 
% plot(model_o1,'k','linewidth',2)
% 
% for j=1:5
%     i=MODELS1.X(j);
%     load(['trial' num2str(i) '.mat'])
%     display(['trial' num2str(i) '.mat'])
%     inv = trial.Invariants.DimGeometric;
%     plot (inv(:,2),'b')
%     
% end

figure
hold on
plot(model_o1,'k','linewidth',2)

for i=1:10
    %i=MODELS1.X(j);
    load(['trial' num2str(i) '.mat'])
    display(['trial' num2str(i) '.mat'])
    inv = trial.Invariants.TimeBased;
    
    t = 1/params.interwaarde:1/(params.interwaarde):1;
    len = length(inv(:,1));
   
    time_interp = interp1(1:len,inv(:,1),t*len)';
 
    plot (time_interp,'b') 
end

load(['trial' num2str(i) '.mat']);
inv = trial.Invariants.DimGeometric;

figure
hold on
subplot(3,2,1)
plot (inv(:,2))
ylabel('\Omega_1')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,2)
plot (inv(:,3))
ylabel('\Omega_2')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,3)
plot (inv(:,4))
ylabel('\Omega_3')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,4)
plot (inv(:,5))
ylabel('V_1')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,5)
plot (inv(:,6))
ylabel('V_2')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,6)
plot (inv(:,7))
ylabel('V_3')
set(get(gca,'YLabel'),'Rotation',0)
