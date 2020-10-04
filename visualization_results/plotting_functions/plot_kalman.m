close all
clear
clc

addpath('../../SmoothedData/run1/invariants_testdata_individual/testmotion/Gewoon1')
addpath('../../Invariants/run1/invariants_testdata_individual/testmotion/Gewoon1')
addpath('../../ScrewTwists/run1/invariants_testdata_individual/testmotion/Gewoon1')

load('smoothed_data1.mat')
load('invariants1.mat')
load('twists1.mat')

smooth=smoothedData.smoothedData.Smooth;
data=smoothedData.smoothedData.Data;
filt=smoothedData.smoothedData.Filtereddata;

figure
subplot(3,1,1)
hold on
plot(data(:,7))
plot(filt(:,7),'k')
plot(smooth(:,7),'r')
ylabel('x')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,1,2)
hold on
plot(data(:,8))
plot(filt(:,8),'k')
plot(smooth(:,8),'r')
ylabel('y')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,1,3)
hold on
plot(data(:,9))
plot(filt(:,9),'k')
plot(smooth(:,9),'r')
ylabel('z')
set(get(gca,'YLabel'),'Rotation',0)

dsmooth=smoothedData.smoothedData.SmoothDot;
figure
subplot(3,1,1)
hold on
plot(dsmooth(:,7),'r')
subplot(3,1,2)
hold on
plot(dsmooth(:,8),'r')
subplot(3,1,3)
hold on
plot(dsmooth(:,9),'r')

ddsmooth=smoothedData.smoothedData.SmoothDotDot;
figure
subplot(3,1,1)
hold on
plot(ddsmooth(:,7),'r')
subplot(3,1,2)
hold on
plot(ddsmooth(:,8),'r')
subplot(3,1,3)
hold on
plot(ddsmooth(:,9),'r')

dddsmooth=smoothedData.smoothedData.SmoothDotDotDot;
figure
subplot(3,1,1)
hold on
plot(dddsmooth(:,7),'r')
subplot(3,1,2)
hold on
plot(dddsmooth(:,8),'r')
subplot(3,1,3)
hold on
plot(dddsmooth(:,9),'r')

twist = twistData.Twists.Twist;
figure
subplot(2,3,1)
hold on
plot (twist(:,1))
subplot(2,3,2)
hold on
plot (twist(:,2))
subplot(2,3,3)
hold on
plot (twist(:,3))
subplot(2,3,4)
hold on
plot (twist(:,4))
subplot(2,3,5)
hold on
plot (twist(:,5))
subplot(2,3,6)
hold on
plot (twist(:,6))

inv = invariants.Invariants.TimeBased;
figure
hold on
subplot(3,2,1)
plot (inv(:,1))
ylabel('\omega_1')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,3)
plot (inv(:,2))
ylabel('\omega_2')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,5)
plot (inv(:,3))
ylabel('\omega_3')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,2)
plot (inv(:,4))
ylabel('v_1')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,4)
plot (inv(:,5))
ylabel('v_2')
set(get(gca,'YLabel'),'Rotation',0)
subplot(3,2,6)
plot (inv(:,6))
ylabel('v_3')
set(get(gca,'YLabel'),'Rotation',0)
