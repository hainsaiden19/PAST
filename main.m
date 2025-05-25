
clear
clc
close all

disp('Simulation Started')

%%% Init Planet attributes %%%%
Planet


%%% Initial conditions for Position & Velocity  %%%%%%%%
alt = 600e3; % 600km alt [metres]
inc = deg2rad(56);
sma = alt + earthRadius;

x0 = alt + earthRadius;
y0 = 0;
z0 = 0;

r = norm([x0; y0; z0]);
circ_vel = sqrt(earth_mu/r);

xdot0 = 0; 
ydot0 = circ_vel*cos(inc); 
zdot0 = circ_vel*sin(inc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Initial conditions for Attitude and Angular Velocity  %%%%%%%%
phi0 = 0;
theta0 = 0;
psi0 = 0;
euler0 = [phi0 theta0 psi0];

quart0 = eul2quat(euler0)';

p0 = -30*((2*pi)/360);
q0 = 20*((2*pi)/360);
r0 = -10*((2*pi)/360);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Propogate Cubesat %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% format: position xyz (1:3), velocity in xyz directions (4:6), 
%         quarternion orientation (7:10), angular velocity (11:13)


%%% Time Window %%%
orbit_period = 2*pi*sqrt((sma^3)/(earth_mu));
num_orbits = 0.5;
tfinal = orbit_period*num_orbits;
tstep = 0.01; % seconds
tout = 0:1:tfinal;

% for plotting
Bxout = 0*tout;
Byout = 0*tout;
Bzout = 0*tout;

state = [x0; y0; z0; xdot0; ydot0; zdot0; quart0; p0; q0; r0];
stateout = zeros(length(tout), length(state));

%%% RK4 numerical integration %%%
for idx = 1:length(tout)
    % keep track of sim
    message = sprintf("%d / %d", tout(idx), max(tout));

    disp(message)


    magfieldcurrent = MagneticField(state(1), state(2), state(3));
    Bxout(idx) = magfieldcurrent(1);
    Byout(idx) = magfieldcurrent(2);
    Bzout(idx) = magfieldcurrent(3);

    % save the state to a library of all states
    stateout(idx, :) = state';
   
    % integrate for next state
    k_1 = Satellite(tout(idx), state);
    k_2 = Satellite(tout(idx)+tstep/2, state+(k_1*tstep/2));
    k_3 = Satellite(tout(idx)+tstep/2, state+(k_2*tstep/2));
    k_4 = Satellite(tout(idx)+tstep, state+(k_3*tstep));

    k = (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4);

    % change current state to new integrated one   
    state = state + k*tstep;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Extract Vector Components %%%
xout = stateout(:, 1);
yout = stateout(:, 2);
zout = stateout(:, 3);
vxout = stateout(:, 4);
vyout = stateout(:, 5);
vzout = stateout(:, 6);
q1out = stateout(:, 7);
q2out = stateout(:, 8);
q3out = stateout(:, 9);
q4out = stateout(:, 10);
pout = stateout(:, 11);
qout = stateout(:, 12);
rout = stateout(:, 13);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Get Euler angles from quarternions %%%
quarternions = stateout(:, 7:10);
eulerangles = quat2eul(quarternions);
phiout = eulerangles(:, 1);
thetaout = eulerangles(:, 2);
psiout = eulerangles(:, 3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pout = pout*(180/(pi));
qout = qout*(180/(pi));
rout = rout*(180/(pi));

mess = sprintf("p: %.5f, q: %.5f, r: %.5f", pout(end), qout(end), rout(end));
disp(mess)


disp('Simulation Complete')


%%% Plotting Orbit %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xaxes = 0:100:2*R;
yaxes = 0:100:2*R;
zaxes = 0:100:2*R;
touthours = tout/3600;

fig1 = figure();
set(fig1, 'color', 'white');
plot3(xout, yout, zout, 'b-', 'LineWidth', 4);
grid on
hold on
surf(Xe, Ye, Ze, 'EdgeColor','none');
title('Cubesat Orbit 3D')
xlabel('X')
ylabel('Y')
zlabel('Z')
plot3(xaxes, 0*xaxes, 0*xaxes, 'r-', LineWidth=2)
plot3(0*yaxes, yaxes, 0*yaxes, 'g-', LineWidth=2)
plot3(0*zaxes, 0*zaxes, zaxes, 'b-', LineWidth=2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%% Plotting Magnetic Field over time %%%%%%%%%%%
fig2 = figure();
set(fig2, color='white')
plot(touthours, Bxout, 'r-')
grid on
hold on
plot(touthours, Byout, 'g-')
plot(touthours, Bzout, 'b-')
ylabel('Magnetic Field (T)') 
xlabel('Time (Hours)')
title('Magnetic Field Components During Orbit')
legend('X', 'Y', 'Z')
xlim([0 max(touthours)])

% fig3 = figure();
% set(fig3, color='white')
% touthours = tout/3600;
% plot(touthours, Bxderivout, 'r-')
% grid on
% hold on
% plot(touthours, Byderivout, 'g-')
% plot(touthours, Bzderivout, 'b-')
% ylabel('Derivative of Magnetic Field') 
% xlabel('Time (Hours)')
% title('Derivative of Magnetic Field Components During Orbit')
% legend('Bx', 'By', 'Bz')
% xlim([0 max(touthours)])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Plot angular velocity over time %%%%%%
fig4 = figure();
set(fig4, color='white')
plot(touthours, pout, 'r-')
hold on
grid on
plot(touthours, qout, 'g-')
plot(touthours, rout, 'b-')
xlabel('time (hours)')
ylabel('Angular Velocity (degrees/s)')
title('Angular Velocity Over Time')
legend('p', 'q', 'r')
xlim([0 max(touthours)])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Plot Attitude Over Time %%%%%%%%%
fig5 = figure();
set(fig5, color='white')
plot(touthours, phiout, 'r-')
hold on
grid on
plot(touthours, thetaout, 'g-')
plot(touthours, psiout, 'b-')
xlabel('time (hours)')
ylabel('Attitude (rads)')
title('Attitude Over Time')
legend('phi', 'theta', 'psi')
xlim([0 max(touthours)])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%