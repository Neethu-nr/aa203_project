clf; clc; format long;
close all;
clear

global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3 

% Wind parameters
global Vw;Vw = 10;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

global N; N = 40; % Scenario
global T; T = 165;
global yf; yf = 0;
global hf; hf = 0;
global x0; x0 = 0;
global y0; y0 = 0;
global h0; h0 = 1000;
global V0; V0 = 100;
global gamma0; gamma0 = 0.26;
global chi0; chi0 = pi;
global chif; chif = 0;


global alphaMax; alphaMax = 25 / 180 * pi;
global muMax; muMax = 90 / 180 * pi;
global Vmin; Vmin = 56;

guess = spline_guess([0;0;1000],[-5000;0;0],5,N,true);

uInit = zeros(2*N+2,1); % Initialization on the control
%uInit(N+1:end) = 0.1 * ones(N,1);
sInit = ones(6*(N+1),1); % Initialization on the state

if false
    sInit(1:6*N+6) = guess;
else
    res = matfile('res.mat'); 
    var = res.var;
    sInit(1:6*N+6) = var((1:6*N+6));
end


varInit = [sInit;uInit];

lb = zeros(8*N+8,1); ub = alphaMax*ones(8*N+8,1); % Lower and upper bounds. For the control: |u| \le uMax
lb(1:6*N+6) = -1e5; ub(1:6*N+6) = 1e5; % For the state x : 0 \le x \le M
lb(2*N+3:3*N+3) = zeros(N+1,1);
lb(7*N+7:end) = -muMax; ub(7*N+7:end) = muMax; % For the state y : 0\le x \le l

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',30000,'ConstraintTolerance',1e-2);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraints,options); % Solving the problem
convergence % = 1, good

save('res.mat','var');

plot_traj(var)
plot3(guess(1:N+1),guess(N+2:2*N+2),guess(2*N+3:3*N+3))
load gong.mat;
sound(y);

figure
plot(var(1:N+1))
figure
plot(var(N+2:2*N+2))
figure
plot(var(2*N+3:3*N+3))
