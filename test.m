clear all; clf; clc; format long;
close all;

global N; N = 20; % Scenario
global T; T = 150.;
global yf; yf = 2000;
global hf; hf = 0;
global x0; x0 = 0;
global y0; y0 = 0;
global h0; h0 = 3000;
global V0; V0 = 100;
global gamma0; gamma0 = 0;
global chi0; chi0 = 0;


global alphaMax; alphaMax = 25 / 180 * pi;
global muMax; muMax = 90 / 180 * pi;


uInit = zeros(2*N,1); % Initialization on the control
uInit(N+1:end) = 0.1 * ones(N,1);
sInit = zeros(6*(N+1),1); % Initialization on the state
sInit(1:N+1) = -[0;cumsum(4200/(N)*ones(N,1))];
A = [0;cumsum(3000/(N)*ones(N,1))];
sInit(2*N+3:3*N+3) = (fliplr(A'))';
varInit = [sInit; uInit];

lb = zeros(8*N+6,1); ub = alphaMax*ones(8*N+6,1); % Lower and upper bounds. For the control: |u| \le uMax
lb(1:6*N+6) = -1e5; ub(1:6*N+6) = 1e5; % For the state x : 0 \le x \le M
lb(7*N+7:end) = -muMax; ub(7*N+7:end) = muMax; % For the state y : 0\le x \le l

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',10000);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraints,options); % Solving the problem
convergence % = 1, good

plot3(var(1:N+1),var(N+2:2*N+2),var(2*N+3:3*N+3))
