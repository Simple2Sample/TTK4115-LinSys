% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert h�sten 2006 av Jostein Bakkeheim
% Oppdatert h�sten 2008 av Arnfinn Aas Eielsen
% Oppdatert h�sten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = 1;







%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]
k_f = 0.713; %Motor Constant [F/V]
J_p = 2*m_p*(l_p)^2;
J_e = m_c*(l_c)^2 + 2*m_p*(l_h)^2;
L_3 = l_h*l_p*k_f
k_1 = l_p * k_f;
k_2 = L_3/J_e;
pole_1 = -9;
pole_2 = -5;

K_pp = (pole_1 * pole_2)/(k_1);  %Elevation Proportional Gain
K_pd = -(pole_1+pole_2)/(k_1);  %Elevation derivational Gain

%%%%%%%%%%%Matrices for Day 2


Vs0 = (l_h*2*m_p*g + l_c*m_c*g)/L_3;
%K = [k_11 k_12 k_13; k_21 k_22 k_23];

A = [0 1 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 1 0 0 0 0; 0 0 1 0 0];
B = [0 0; 0 k_1; k_2 0; 0 0; 0 0];
p = [-0.7;-1.6;-1.9];
Q = diag([100;100;100;100;100]);
R = 1.2*eye(2);
K = lqr(A,B,Q,R)
F = [K(1,1) K(1,3); K(2,1) K(2,3)]