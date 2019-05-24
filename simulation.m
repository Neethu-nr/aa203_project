close all
clear
clc

% Global values
global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3 

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

% Control values
alpha = 0.05; % angle of attack in radians
mu = 0.5 ;   % rolling angle in radians

tspan = [0 5];

% initial conditions
x0 = 0; y0 = 0; h0 = 100;
V0 = 100; % m/s
gamma0 = 0.26; % radians
chi0 = 0; % radians
S0 = [x0 ; y0; h0; 
      V0; gamma0; chi0];
 
[t,S] = ode45(@(t,S) aircraft_dynamics(t,S,alpha,mu),tspan,S0);
plot3(S(:,1),S(:,2),S(:,3))