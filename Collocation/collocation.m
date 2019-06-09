clf; clc; format long;
close all;
clear

load_data = false;
notification = false;

%% Initialize varialbes

global W;W = 91200; % Weight of aircraft in kg
global Surface_area;Surface_area =27.87; % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11; % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6; %Radius of earth in m
global mE;mE = 5.972*1e24; %mass of earth in kg

% Wind parameters
global Vw;Vw = 0; % Magnitude of wind in m/s
global chiw;chiw = 0; % Direction of wind in rad

global N; N = 40; % Scenarios
global T; T = 175; %Total time

%Initial constraints
global x0; x0 = 0;
global y0; y0 = 0;
global h0; h0 = 1000;
global V0; V0 = 100;
global gamma0; gamma0 = 0.26;
global chi0; chi0 = pi;

%Final constraints
global yf; yf = 0;
global hf; hf = 0;
global chif; chif = 0;

%Control constraints
global alphaMax; alphaMax = 25 / 180 * pi;
global muMax; muMax = 90 / 180 * pi;

global Vmin; Vmin = 56;

guess = spline_guess([0;0;1000],[-15000;0;0],2,N,true);

uInit = zeros(2*N+2,1); % Initialization on the control
sInit = ones(6*(N+1),1); % Initialization on the state

%Initialize state/control
if ~load_data
    sInit(1:6*N+6) = guess;
    disp('Guess solution')
else
    disp('Load data')
    res = matfile('res.mat'); 
    var = res.var;
    sInit(1:6*N+6) = var((1:6*N+6));
    uInit(:) = var(6*N+7:end);
end

varInit = [sInit;uInit];

%% Bounds on variables
lb = zeros(8*N+8,1); ub = alphaMax*ones(8*N+8,1);
lb(1:6*N+6) = -1e5; ub(1:6*N+6) = 1e5; 
lb(2*N+3:3*N+3) = zeros(N+1,1);
lb(7*N+8:end) = -muMax; ub(7*N+8:end) = muMax; 

%% Optimizing

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',100000,'ConstraintTolerance',1e-3);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraints,options); % Solving the problem
convergence % = 1, good

save('res.mat','var','N','T');

%% Plot trajectory

plot_traj(var)
[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);
t = 0:T/N:T;
figure
plot(x,y)
xlabel('x')
ylabel('y')

figure
plot(t,h)
xlabel('time')
title('h')

figure
plot(t,V)
xlabel('time')
title('V')

figure
plot(t,gamma)
xlabel('time')
title('\gamma')

figure
plot(t,chi)
xlabel('time')
title('\chi')


%% Notification

if notification
    load gong.mat;
    sound(y);
end
