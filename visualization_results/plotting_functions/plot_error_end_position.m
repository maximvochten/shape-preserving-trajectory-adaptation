function plot_error_end_position(time_meas,error_end_position_x,error_end_position_y,error_end_position_z)

error_end_position_x(isnan(error_end_position_x))=0;
error_end_position_y(isnan(error_end_position_y))=0;
error_end_position_z(isnan(error_end_position_z))=0;

% plot error
figure(20);
subplot(3,1,1)
hold on
plot(time_meas(3:end),error_end_position_x,'b','linewidth',1);
ylabel('x')
subplot(3,1,2)
hold on
plot(time_meas(3:end),error_end_position_y,'b','linewidth',1);
ylabel('y')
subplot(3,1,3)
hold on
plot(time_meas(3:end),error_end_position_z,'b','linewidth',1);
ylabel('z')

totalerror =  ( norm(error_end_position_x) + norm(error_end_position_y) + norm(error_end_position_z) ) / length(error_end_position_x)
