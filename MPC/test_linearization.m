clear
clc

global W;W = 91200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg       % density of air in kg/m3 

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

%%


x0 = [0,0,1000,100,0.26,pi]';

x = [0,0,1100,110,0.26,pi]';

dt = 0.01;

u0 = [0.1;0.2];
u = [0.1;0.4];

f0 = aircraft_dynamics(x0,u0(1),u0(2));
A = A_jac(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),u0(1),u0(2));
B =B_jac(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),u0(1),u0(2));

f = aircraft_dynamics(x,u(1),u(2));

norm(f0 + A*(x-x0) + B*(u-u0) - f)
norm(f-f0)

%% Trajectory test

close all

alpha = 0.2; % angle of attack in radians
mu = 0.3;   % rolling angle in radians

dt = 0.01;

tspan = 0:dt:10;
%tspan = [0 100];

% initial conditions
x0 = 0; y0 = 0; h0 = 8000;
V0 = 100; % m/s
gamma0 = 0.26; % radians
chi0 = 0; % radians
S0 = [x0 ; y0; h0; 
      V0; gamma0; chi0];
 
S = zeros(length(tspan),6);
S(1,:) = S0;
test = zeros(length(1:4:length(tspan)),6);
test(1,:) = S0;
test1 = zeros(length(1:4:length(tspan))-1,2);
test1(1,:) = [alpha,mu];

for i = 1:length(tspan)-1
   
    S(i+1,:) = S(i,:) + dt * aircraft_dynamics(S(i,:),alpha+i*0.00001,mu)';
    
end


i = 0;
for idx = 1:4:length(tspan)-1
   
    i = i + 1;
    A = A_jac(S(idx,1),S(idx,2),S(idx,3),S(idx,4),S(idx,5),S(idx,6),test1(i,1),test1(i,2));
    B = B_jac(S(idx,1),S(idx,2),S(idx,3),S(idx,4),S(idx,5),S(idx,6),test1(i,1),test1(i,2));
    f0 = aircraft_dynamics(S(idx,:),test1(i,1),test1(i,2));
    f_tilde = f0 + A*(test(i,:)-S(idx,:))' + B*(test1(i,:)-[alpha+idx*0.00001,mu])';
    test(i+1,:) = test(i,:) + 4*dt*f_tilde';
    test1(i+1,:) = test1(i,:) + [idx*0.00001,0];
end

figure
hold on
plot3(S(:,1),S(:,2),S(:,3),'')
plot3(test(:,1),test(:,2),test(:,3),'--')