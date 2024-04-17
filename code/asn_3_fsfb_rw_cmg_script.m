% Mike Kabot, AERO560 Assignment 3 - Reaction Wheels
clear; close all; clc

%% Problem 2 - RWs
mu = 398600;

% COEs
h = 55759;                               % km2/s
e = 0.001;                                      % UNITLESS
RAAN = 10;                                   % deg
i = 42;                                      % deg
w = 22;                                      % deg
theta = 0;                                  % deg
angles = [RAAN i, w];                       % for rotation sequence

% run COEs2RV to get state vectors
[R_i,V_i] = COEs2RV(h, e, theta, mu, angles);   % works

% Initial attitude - Define the body axes as being initially aligned with 
% F_LVLH and the long side of the spacecraft is in the nadir direction.

% define J
m = 500;
J = m*1/12*[(1.5^2 + 3^2) 0 0; 0 (1.5^2 + 3^2) 0; 0 0 (1.5^2 + 1.5^2)];

% reaction wheel params
theta = 57;  % degrees
S_1 = [sind(theta); 0; cosd(theta)];
T_1 = [-cosd(theta); 0; sind(theta)];
G_1 = cross(S_1, T_1);

S_3 = [-sind(theta); 0; cosd(theta)];
T_3 = [cosd(theta); 0; sind(theta)];
G_3 = cross(S_3, T_3);

S_2 = [0; sind(theta); cosd(theta)];
T_2 = [0; -cosd(theta); sind(theta)];
G_2 = cross(S_2, T_2);

S_4 = [0; -sind(theta); cosd(theta)];
T_4 = [0; cosd(theta); sind(theta)];
G_4 = cross(S_4, T_4);


% plot check wheel frames
pyr = figure();
hold on, axis equal
plot3([1 1+S_1(1)], [0 S_1(2)], [0 S_1(3)])
plot3([1 1+T_1(1)], [0 T_1(2)], [0 T_1(3)])
% plot3([1 1+G_1(1)], [0 G_1(2)], [0 G_1(3)])

plot3([0 S_2(1)], [1 1+S_2(2)], [0 S_2(3)])
plot3([0 T_2(1)], [1 1+T_2(2)], [0 T_2(3)])
% plot3([0 G_2(1)], [1 1+G_2(2)], [0 G_2(3)])

plot3([-1 -1+S_3(1)], [0 S_3(2)], [0 S_3(3)])
plot3([-1 -1+T_3(1)], [0 T_3(2)], [0 T_3(3)])
% plot3([-1 -1+G_3(1)], [0 G_3(2)], [0 G_3(3)])

plot3([0 S_4(1)], [-1 -1+S_4(2)], [0 S_4(3)])
plot3([0 T_4(1)], [-1 -1+T_4(2)], [0 T_4(3)])
% plot3([0 G_4(1)], [-1 -1+G_4(2)], [0 G_2(3)])
view(45, 30)
title("Pyramid View")

% coordinate transforms
A_S = [S_1, S_2, S_3, S_4];
A_S_dag = pinv(A_S);

A_T = [T_1, T_2, T_3, T_4];
A_G = [G_1, G_2, G_3, G_4];


% attitude
I_ws = 1.2;

% define C_LVLH_ECI_i
x_LVLH_ECI_i = V_i/norm(V_i);                                   % x body aligned with v
z_LVLH_ECI_i = -R_i/norm(R_i);                                  % z body radial inward
y_LVLH_ECI_i = cross(z_LVLH_ECI_i, x_LVLH_ECI_i);               % y body is in cross track

C_LVLH_ECI_i = [x_LVLH_ECI_i, y_LVLH_ECI_i, z_LVLH_ECI_i]';      % defines C_LVLH_ECI_i 

% initial LVLH attitude
eps_b_LVLH_i = [0.5;0.5;0.5]; % givens
eta_b_LVLH_i = 0.5;

C_b_LVLH_i = quat_RotMat(eps_b_LVLH_i, eta_b_LVLH_i);

[phi_b_LVLH_i, theta_b_LVLH_i, psi_b_LVLH_i] = eulerAngles(C_b_LVLH_i);

% initial ECI attitude
C_b_ECI_i = C_b_LVLH_i*C_LVLH_ECI_i;

[phi_b_ECI_i, theta_b_ECI_i, psi_b_ECEI_i] = eulerAngles(C_b_ECI_i);

[eps_b_ECI_i, eta_b_ECI_i] = quaternionParam(C_b_ECI_i);

% inital ECI angular velocity
w_b_ECI_i = zeros(3,1);                  % rad/s [3x1]

% calculate mean motion for T_d
T = 86400;                                  % s (assume perfect GEO orbit)
n = 2*pi/T;                                 % rad/s

% gain calculation
zeta = 0.7;
w_n = 0.5;

K_p = 2*J*w_n^2;
K_d = J*2*zeta*w_n;


%% displays
disp("As = ")
disp(A_S)
disp("At = ")
disp(A_T)
disp("Ag = ")
disp(A_G)
disp("Platform Inertia = ")
disp(A_S*diag(I_ws*ones(1,4)))
disp("Spacecraft Inerta = ")
disp(J)
disp("Kp = ")
disp(K_p)
disp("Kd = ")
disp(K_d)

%% Run Simulink (RW)

out = sim("asn_3_fsfb_rw.slx", 40);

%% Plot

% b_ECI
figure() % q_b_ECI - good
hold on, grid on
plot(out.tout, out.eps_b_ECI(:,1))
plot(out.tout, out.eps_b_ECI(:,2))
plot(out.tout, out.eps_b_ECI(:,3))
plot(out.tout, out.eta_b_ECI)
legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
xlabel("time [s]")
title("q_{b,ECI}")

figure() % eul_b_ECI - good
hold on, grid on
plot(out.tout, out.eul_b_ECI(:,1))
plot(out.tout, out.eul_b_ECI(:,2))
plot(out.tout, out.eul_b_ECI(:,3))
legend("\phi","\theta","\psi")
xlabel("time [s]")
ylabel("angle [rad]")
title("eul_{b,ECI}")

figure() % w_b_ECI - good
hold on, grid on
plot(out.tout, out.w_b_ECI(:,1))
plot(out.tout, out.w_b_ECI(:,2))
plot(out.tout, out.w_b_ECI(:,3))
legend("\omega_1","\omega_2","\omega_3")
xlabel("time [s]")
ylabel("angle/s [rad/s]")
title("\omega_{b,ECI}")

% F_LVLH_ECI
figure() % q_LVLH_ECI 
hold on, grid on
plot(out.tout, out.eps_LVLH_ECI(:,1)) 
plot(out.tout, out.eps_LVLH_ECI(:,2))
plot(out.tout, out.eps_LVLH_ECI(:,3))
plot(out.tout, out.eta_LVLH_ECI)
legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
xlabel("time [s]")
title("q_{LVLH,ECI}")

figure() % eul_LVLH_ECI
hold on, grid on
plot(out.tout, out.eul_LVLH_ECI(:,1))
plot(out.tout, out.eul_LVLH_ECI(:,2))
plot(out.tout, out.eul_LVLH_ECI(:,3))
legend("\phi","\theta","\psi")
xlabel("time [s]")
ylabel("angle [rad]")
title("eul_{LVLH,ECI}")

figure() % w_b_LVLH
hold on, grid on
plot(out.tout, out.w_LVLH_ECI(1,:))
plot(out.tout, out.w_LVLH_ECI(2,:))
plot(out.tout, out.w_LVLH_ECI(3,:))
legend("\omega_1","\omega_2","\omega_3")
xlabel("time [s]")
ylabel("angle/s [rad/s]")
title("\omega_{LVLH,ECI}")

% b_LVLH
figure() % q_b_LVLH 
hold on, grid on
plot(out.tout, out.eps_b_LVLH(:,1))
plot(out.tout, out.eps_b_LVLH(:,2))
plot(out.tout, out.eps_b_LVLH(:,3))
plot(out.tout, out.eta_b_LVLH)
legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
xlabel("time [s]")
title("q_{b,LVLH}")

figure() % eul_b_LVLH
hold on, grid on
plot(out.tout, out.eul_b_LVLH(:,1))
plot(out.tout, out.eul_b_LVLH(:,2))
plot(out.tout, out.eul_b_LVLH(:,3))
legend("\phi","\theta","\psi")
xlabel("time [s]")
ylabel("angle [rad]")
title("eul_{b,LVLH}")

figure() % w_b_LVLH
hold on, grid on
plot(out.tout, out.w_b_LVLH(1,:))
plot(out.tout, out.w_b_LVLH(2,:))
plot(out.tout, out.w_b_LVLH(3,:))
legend("\omega_1","\omega_2","\omega_3")
xlabel("time [s]")
ylabel("angle/s [rad/s]")
title("\omega_{b,LVLH}")

figure() % sensor angle
plot(out.tout,rad2deg(out.sensor_theta))
xlabel("time [s]")
ylabel("\theta_{sensor} [degrees]")
title("\theta_{sensor}")

figure() % Omega
plot(out.Omega)
xlabel("time [s]")
ylabel("\theta_{sensor} [degrees]")
title("\theta_{sensor}")

% %% Problem 3 - CMGs
% 
% % weheel parameters
% m = 4.5; % kg
% h = 0.05; % m
% r = 0.2; % m
% Omega = [800;800;800;800]; % rad/s
% 
% I_cs = diag(1/2*m*r^2*ones(1, 4));
% I_ct = diag(1/4*m*r^2 + 1/12*m*h^2*ones(1,4));
% 
% %% displays
% disp("As = ")
% disp(A_S)
% disp("At = ")
% disp(A_T)
% disp("Ag = ")
% disp(A_G)
% disp("Platform Inertia = ")
% disp(A_S*I_cs+ A_T*I_ct)
% disp("Spacecraft Inerta = ")
% disp(J)
% disp("Kp = ")
% disp(K_p)
% disp("Kd = ")
% disp(K_d)
% 
% 
% %% Run Simulink (RW)
% 
% out = sim("asn_3_fsfb_cmg.slx", 40);
% 
% %% Plot
% 
% % b_ECI
% figure() % q_b_ECI - good
% hold on, grid on
% plot(out.tout, out.eps_b_ECI(:,1))
% plot(out.tout, out.eps_b_ECI(:,2))
% plot(out.tout, out.eps_b_ECI(:,3))
% plot(out.tout, out.eta_b_ECI)
% legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
% xlabel("time [s]")
% title("q_{b,ECI}")
% 
% figure() % eul_b_ECI - good
% hold on, grid on
% plot(out.tout, out.eul_b_ECI(:,1))
% plot(out.tout, out.eul_b_ECI(:,2))
% plot(out.tout, out.eul_b_ECI(:,3))
% legend("\phi","\theta","\psi")
% xlabel("time [s]")
% ylabel("angle [rad]")
% title("eul_{b,ECI}")
% 
% figure() % w_b_ECI - good
% hold on, grid on
% plot(out.w_b_ECI)
% legend("\omega_1","\omega_2","\omega_3")
% xlabel("time [s]")
% ylabel("angle/s [rad/s]")
% title("\omega_{b,ECI}")
% 
% % F_LVLH_ECI
% figure() % q_LVLH_ECI 
% hold on, grid on
% plot(out.tout, out.eps_LVLH_ECI(:,1)) 
% plot(out.tout, out.eps_LVLH_ECI(:,2))
% plot(out.tout, out.eps_LVLH_ECI(:,3))
% plot(out.tout, out.eta_LVLH_ECI)
% legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
% xlabel("time [s]")
% title("q_{LVLH,ECI}")
% 
% figure() % eul_LVLH_ECI
% hold on, grid on
% plot(out.tout, out.eul_LVLH_ECI(:,1))
% plot(out.tout, out.eul_LVLH_ECI(:,2))
% plot(out.tout, out.eul_LVLH_ECI(:,3))
% legend("\phi","\theta","\psi")
% xlabel("time [s]")
% ylabel("angle [rad]")
% title("eul_{LVLH,ECI}")
% 
% figure() % w_b_LVLH
% hold on, grid on
% plot(out.w_LVLH_ECI)
% legend("\omega_1","\omega_2","\omega_3")
% xlabel("time [s]")
% ylabel("angle/s [rad/s]")
% title("\omega_{LVLH,ECI}")
% 
% % b_LVLH
% figure() % q_b_LVLH 
% hold on, grid on
% plot(out.tout, out.eps_b_LVLH(:,1))
% plot(out.tout, out.eps_b_LVLH(:,2))
% plot(out.tout, out.eps_b_LVLH(:,3))
% plot(out.tout, out.eta_b_LVLH)
% legend("\epsilon_1","\epsilon_2","\epsilon_3","\eta")
% xlabel("time [s]")
% title("q_{b,LVLH}")
% 
% figure() % eul_b_LVLH
% hold on, grid on
% plot(out.tout, out.eul_b_LVLH(:,1))
% plot(out.tout, out.eul_b_LVLH(:,2))
% plot(out.tout, out.eul_b_LVLH(:,3))
% legend("\phi","\theta","\psi")
% xlabel("time [s]")
% ylabel("angle [rad]")
% title("eul_{b,LVLH}")
% 
% figure() % w_b_LVLH
% hold on, grid on
% plot(out.w_b_LVLH)
% legend("\omega_1","\omega_2","\omega_3")
% xlabel("time [s]")
% ylabel("angle/s [rad/s]")
% title("\omega_{b,LVLH}")
% 
% figure() % sensor angle
% plot(out.tout,rad2deg(out.sensor_theta))
% xlabel("time [s]")
% ylabel("theta_{sensor} [degrees]")
% title("theta_{sensor}")
% 
% figure() % gamma
% plot(out.gamma)
% xlabel("time [s]")
% ylabel("gamma [degrees]")
% title("gamma")
% 
% 
