close all
clear
clc


addpath('Trials_runa/trials_special_motions/linear_motion/gewoon1')
addpath('Leds_runa/leds_special_motions/linear_motion/gewoon1')
addpath('Twist_runa/twists_special_motions/linear_motion/gewoon1')

number = 1;

load(['trial' num2str(number) '.mat'])
load(['leds' num2str(number) '.mat'])
load(['twist' num2str(number) '.mat'])

inv = trial.Invariants.AltTimeBased;
figure
subplot(3,2,1)
hold on
plot (inv(:,1))
subplot(3,2,2)
hold on
plot (inv(:,2))
subplot(3,2,3)
hold on
plot (inv(:,3))
subplot(3,2,4)
hold on
plot (inv(:,4))
subplot(3,2,5)
hold on
plot (inv(:,5))
subplot(3,2,6)
hold on
plot (inv(:,6))

smooth=ledsData.Leds.Smooth;
data=ledsData.Leds.Data;
filt=ledsData.Leds.Filtereddata;
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

dsmooth=ledsData.Leds.SmoothDot;
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

ddsmooth=ledsData.Leds.SmoothDotDot;
figure
subplot(3,1,1)
hold on
plot(ddsmooth(:,1),'r')
subplot(3,1,2)
hold on
plot(ddsmooth(:,2),'r')
subplot(3,1,3)
hold on
plot(ddsmooth(:,3),'r')

dddsmooth=ledsData.Leds.SmoothDotDotDot;
figure
subplot(3,1,1)
hold on
plot(dddsmooth(:,1),'r')
subplot(3,1,2)
hold on
plot(dddsmooth(:,2),'r')
subplot(3,1,3)
hold on
plot(dddsmooth(:,3),'r')

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