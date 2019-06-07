close all;
clear
clc

% Global values
global W;W = 91200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

res = matfile('res.mat');
global N;N=res.N;
global T;T=res.T;

% Nominal controls and trajectory
[xnom,ynom,hnom,Vnom,gammanom,chinom,alphanom,munom]=varToState(res.var);


%spline;

dt=0.1;
t=0:(T/N):T; 
t_desired=0:dt:T;
xnom=(spline(t,xnom,t_desired))';
ynom=(spline(t,ynom,t_desired))';
hnom=(spline(t,hnom,t_desired))';
Vnom=(spline(t,Vnom,t_desired))';
gammanom=(spline(t,gammanom,t_desired))';
chinom=(spline(t,chinom,t_desired))';
alphanom=(spline(t,alphanom,t_desired))';
munom=(spline(t,munom,t_desired))';

nomState=[xnom';ynom';hnom';Vnom';gammanom';chinom'];
nomControl=[alphanom';munom'];

% define all A and B matrices
Acurr = zeros(6,6,length(t_desired));
Bcurr = zeros(6,2,length(t_desired));
size(Acurr)

for i=1:length(t_desired)
    if mod(i,100)==0
       disp(i) 
    end
    Acurr(:,:,i)=A_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    Bcurr(:,:,i)=B_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
end

disp("Done")

save('nom_traj.mat','Acurr','Bcurr','nomState','nomControl','t_desired','dt')

%% Plot
figure;
hold on;
plot3(nomState(1,1:end),nomState(2,1:end),nomState(3,1:end),'m--');
airplane_simulator(@aircraft_dynamics,dt,nomControl,nomState(:,1),dt)


%% plot linearized dynamics
% xlin=zeros(size(nomState));
% xlin(:,1) = nomState(:,1);
% 
% for i = 1:length(t_desired)-1
%    
%     x_bar_1 = nomState(:,i+1);
%     x_bar = nomState(:,i);
%     u_bar = nomControl(:,i);
%     f_bar = aircraft_dynamics(x_bar,u_bar(1),u_bar(2));
%     A_bar = Acurr(:,:,i);
%     B_bar = Bcurr(:,:,i);
%     G_bar=x_bar_1-x_bar+dt*(f_bar-A_bar*x_bar-B_bar*u_bar);
% 
%     xlin(:,i+1)=G_bar+dt*A_bar*xlin(:,i)+xlin(:,i)+ dt*B_bar*u_bar;
%     
% end
% 
% plot3(xlin(1,1:end),xlin(2,1:end),xlin(3,1:end),'b-.')