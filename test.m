clf; clc; format long;
close all;
clear

global N; N = 35; % Scenario
global T; T = 170;
global yf; yf = 0;
global hf; hf = 0;
global x0; x0 = 0;
global y0; y0 = 0;
global h0; h0 = 1000;
global V0; V0 = 100;
global gamma0; gamma0 = 0;
global chi0; chi0 = pi;


global alphaMax; alphaMax = 25 / 180 * pi;
global muMax; muMax = 90 / 180 * pi;

guess = spline_guess([0;0;1000],[5000;0;0],5,N);

uInit = zeros(2*N,1); % Initialization on the control
%uInit(N+1:end) = 0.1 * ones(N,1);
sInit = ones(6*(N+1),1); % Initialization on the state
sInit(1:6*N+6) = guess;
varInit = [sInit;uInit];

lb = zeros(8*N+6,1); ub = alphaMax*ones(8*N+6,1); % Lower and upper bounds. For the control: |u| \le uMax
lb(1:6*N+6) = -1e5; ub(1:6*N+6) = 1e5; % For the state x : 0 \le x \le M
lb(2*N+3:3*N+3) = zeros(N+1,1);
lb(7*N+7:end) = -muMax; ub(7*N+7:end) = muMax; % For the state y : 0\le x \le l

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',30000);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraints,options); % Solving the problem
convergence % = 1, good

hold on
plot3(var(1:N+1),var(N+2:2*N+2),var(2*N+3:3*N+3))
plot3(guess(1:N+1),guess(N+2:2*N+2),guess(2*N+3:3*N+3))