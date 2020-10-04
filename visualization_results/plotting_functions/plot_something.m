addpath('./testdata')

load('leds1.mat')

smooth=ledsData.Leds.Smooth;
data=ledsData.Leds.Data;
filt=ledsData.Leds.P_filt;

dsmooth=ledsData.Leds.SmoothDot;
ddsmooth=ledsData.Leds.SmoothDotDot;
dddsmooth=ledsData.Leds.SmoothDotDotDot;

% 
% figure
% subplot(3,1,1)
% hold on
% plot(data(:,1))
% plot(filt(:,1),'k')
% plot(smooth(:,1),'r')
% subplot(3,1,2)
% hold on
% plot(data(:,2))
% plot(filt(:,2),'k')
% plot(smooth(:,2),'r')
% subplot(3,1,3)
% hold on
% plot(data(:,3))
% plot(filt(:,3),'k')
% plot(smooth(:,3),'r')


figure
subplot(3,1,1)
hold on
plot(dsmooth(:,1),'r')
subplot(3,1,2)
hold on
plot(dsmooth(:,2),'r')
subplot(3,1,3)
hold on
plot(dsmooth(:,3),'r')


% figure
% subplot(3,1,1)
% hold on
% plot(ddsmooth(:,1),'r')
% subplot(3,1,2)
% hold on
% plot(ddsmooth(:,4),'r')
% subplot(3,1,3)
% hold on
% plot(dddsmooth(:,3),'r')

ddsmooth=ledsData.Leds.SmoothDotDot;
dddsmooth=ledsData.Leds.SmoothDotDotDot;

figure
subplot(3,1,1)
hold on
plot(data(:,1))
plot(filt(:,1),'k')
plot(smooth(:,1),'r')
subplot(3,1,2)
hold on
plot(data(:,2))
plot(filt(:,2),'k')
plot(smooth(:,2),'r')
subplot(3,1,3)
hold on
plot(data(:,3))
plot(filt(:,3),'k')
plot(smooth(:,3),'r')

load('trial.mat')
inv = trial.Invariants.DimGeometric;
% figure
% subplot(3,2,1)
% hold on
% plot (inv(:,1))
% subplot(3,2,2)
% hold on
% plot (inv(:,2))
% subplot(3,2,3)
% hold on
% plot (inv(:,3))
% subplot(3,2,4)
% hold on
% plot (inv(:,4))
% subplot(3,2,5)
% hold on
% plot (inv(:,5))
% subplot(3,2,6)
% hold on
% plot (inv(:,6))
% Use this if geometric invariants
figure
subplot(3,2,1)
hold on
plot (inv(:,2))
subplot(3,2,2)
hold on
plot (inv(:,3))
subplot(3,2,3)
hold on
plot (inv(:,4))
subplot(3,2,4)
hold on
plot (inv(:,5))
subplot(3,2,5)
hold on
plot (inv(:,6))
subplot(3,2,6)
hold on
plot (inv(:,7))

figure
subplot(1,2,1)
hold on
plot (inv(:,1))
subplot(1,2,2)
hold on
plot (inv(:,8))
