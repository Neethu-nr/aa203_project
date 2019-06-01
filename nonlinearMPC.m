
if ~mpcchecktoolboxinstalled('optim')
    disp('Optimization Toolbox is required to run this example.')
    return
end

clear all;clc;
%% Global var
global W;W = 9200;                        % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;  % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;           % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;                % in radius of earth in m
global mE;mE = 5.972*1e24;                % mass earth in kg
global rho_air;rho_air = 1.225 ;          % density of air in kg/m3 

% Wind parameters
global Vw;Vw = 10;                        % Magnitude of wind in m/s
global chiw;chiw = 0;                     % Direction of wind in rad

global N; N = 10;                         % Scenario
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

S0 =[x0;y0;h0;V0;gamma0;chi0];

%% Local var
n_states = 6;
n_output = 6;
n_input = 2;    
Ts = 4;            % sample time
pred_t = 5;         % prediction horizon
control_t = 155;     % control horizon
sim_t = 20;         % simulation time
%% create a nonlinear MPC controller

nlobj = nlmpc(n_states, n_output, n_input);
nlobj.Ts = Ts;
nlobj.Model.NumberOfParameters = 1;     % Ts
nlobj.PredictionHorizon = pred_t;
nlobj.ControlHorizon = control_t;
nlobj.Model.IsContinuousTime = false;
nlobj.Model.StateFcn = "plane";          % non-linear plant model
nlobj.Model.OutputFcn = @(x,u,Ts) [x];   % plant outputs are the first and third state in the model
% nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0 0 0;
%                                       0 1 0 0 0 0;
%                                       0 0 1 0 0 0;
%                                       0 0 0 1 0 0;
%                                       0 0 0 0 1 0;
%                                       0 0 0 0 0 1];    % analytical Jacobian functions

% MPC tuning weights.
nlobj.Weights.OutputVariables = 3*ones(6,6);
nlobj.Weights.ManipulatedVariablesRate = 0.1*ones(6,2);

%% constraints
% state constraint

nlobj.OV(1).Min = -1000;
nlobj.OV(1).Max = 6000;

nlobj.OV(2).Min = -2000;
nlobj.OV(2).Max = 2000;

nlobj.OV(3).Min = 0;
nlobj.OV(3).Max = h0 + 1000;


% control constraints
nlobj.MV(1).Min = 0;
nlobj.MV(1).Max = alphaMax;
nlobj.MV(2).Min = -muMax;
nlobj.MV(2).Max = muMax;
    
%% Checking if everything is setup OK
x0 = S0;
mv = [0.4;0.4];
validateFcns(nlobj, x0, mv, [], {Ts});


% Initialize control
mv(1) = 0;
mv(2) = 0;

% Loading reference trajectory from collocation method
res = matfile('res.mat'); 
T = 155;
N = 40;
global reference_traj; 
reference_traj = ones(N+1, n_states);
reference_traj(:,1) = res.var(1:N+1,1);
reference_traj(:,2) = res.var(N+2:2*N+2,1);
reference_traj(:,3) = res.var(2*N+3:3*N+3,1);
reference_traj(:,4) = res.var(3*N+4:4*N+4,1);
reference_traj(:,5) = res.var(4*N+5:5*N+5,1);
reference_traj(:,6) = res.var(5*N+6:6*N+6,1);

% compute optimal control moves at each control interval.
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

% Run the simulation for 20 seconds.
hbar = waitbar(0,'Simulation Progress');
xHistory = S0;
y=S0;
for ct = 1:(sim_t/Ts)
    % Set references
    yref = get_reference_trajectory(ct-1);
    % Compute optimal control moves
    [mv,nloptions,info] = nlmpcmove(nlobj,y,mv,yref,[],nloptions);
    % Predict prediction model states for the next iteration
    % predict(EKF, [mv; Ts]);
    % Implement first optimal control move and update plant states.
    x = plane(x,mv,Ts);
    % Generate sensor data with some white noise
    y = x + randn(6,1)*0.01;
    % Save plant states for display.
    xHistory = [xHistory x]; 
    waitbar(ct*Ts/20,hbar);
end
close(hbar);
nloptions.Parameters = {Ts};