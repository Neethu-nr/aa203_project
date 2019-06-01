close all;
clear
clc

% Global values
global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global N;N=40;

% Wind parameters
global Vw;Vw = 10;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

res = matfile('res.mat');

% Nominal controls and trajectory
[xnom,ynom,hnom,Vnom,gammanom,chinom,alphanom,munom]=varToState(res.var);


%spline;

dt=0.01;
t=0:(155/40):155; 
t_desired=0:dt:155;
xnom=(spline(t,xnom,t_desired))';
ynom=(spline(t,ynom,t_desired))';
hnom=(spline(t,hnom,t_desired))';
Vnom=(spline(t,Vnom,t_desired))';
gammanom=(spline(t,gammanom,t_desired))';
chinom=(spline(t,chinom,t_desired))';
alphanom=(spline(t,alphanom,t_desired))';
munom=(spline(t,munom,t_desired))';

nomSate=[xnom';ynom';hnom';Vnom';gammanom';chinom'];
nomControl=[alphanom';munom'];

% define all A and B matrices
Acurr = zeros(6,6,length(t_desired));
Bcurr = zeros(6,2,length(t_desired));

for i=1:length(t_desired)
    Acurr(:,:,i)=A_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    Bcurr(:,:,i)=B_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
end

save('nom_traj.mat','Acurr','Bcurr','nomSate','nomControl','t_desired','dt')
