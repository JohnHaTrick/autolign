% Plot Graph
set(gcf,'defaultLineLineWidth',2)

figure(1);clf;
subplot(611);
plot(T,VX_A);
grid on;
ylabel('Vx Speed [m/s]');

subplot(612);
plot(T,VY_A);
grid on;
ylabel('Vy Speed [m/s]');

subplot(613);
plot(T,YAWRATE_A);
grid on;
ylabel('Yaw Rate [rad/s]');

subplot(614);
plot(T,PSI_A);
grid on;
ylabel('Heading Angle [rad]');

subplot(615);
plot(T,FX1);
hold all;
plot(T,FX2);
plot(T,FX3);
plot(T,FX4);
grid on;
legend({'Fx1','Fx2','Fx3','Fx4'});
ylabel('Long. Force [N]');

subplot(616);
plot(T,FY1_A);
hold all;
plot(T,FY2_A);
plot(T,FY3_A);
plot(T,FY4_A);
grid on;
legend({'Fy1','Fy2','Fy3','Fy4'});
ylabel('Lat. Force [N]');
xlabel('Time [s]');

figure(2);clf;
plot(E_A, N_A);
axis equal;
grid on;