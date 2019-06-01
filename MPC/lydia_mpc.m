close all;
clear
clc

% Global values
global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3
global N;N=40;

% Wind parameters
global Vw;Vw = 10;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

res = matfile('res.mat');

% Nominal controls and trajectory
[xnom,ynom,hnom,Vnom,gammanom,chinom,alphanom,munom]=varToState(res.var);


%spline;
t=0:(155/40):155; 
t_desired=0:0.1:155;
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
for i=1:length(t_desired)
    Acurr(:,:,i)=A_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    Bcurr(:,:,i)=B_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    
end

dt=0.01;
iterations=50;

cvx_quiet True

num_steps = 10;

aircraft_state = zeros(6,num_steps+1);
aircraft_state(:,1) = nomSate(:,1);
control_dt = 0.01;

Q = diag([10 10 10 10 1 1]);
Qhalf = sqrtm(Q);

for step = 0:num_steps-1
    
    step+1
    
    stateCost=0;
    controlCost=0;

    cvx_begin
    variables xtraj(6,iterations) utraj(2,iterations-1)
    for i=1:iterations
        %weight=zeros(6,6);
        %weight(3,3)=3;
        %weight=weight+eye(6);
        stateCost=stateCost+norm(Qhalf*(xtraj(:,i)-nomSate(:,i+step)),'fro')-xtraj(4,i);
        %if i>=2
        %controlCost=controlCost+norm(eye(2)*(utraj(:,i-1)-nomControl(:,i-1+step)));
        %end
    end
    minimize stateCost+controlCost
    %initial conditions
    xtraj(:,1)==aircraft_state(:,1+step)

    for i=2:iterations
        Gnom=nomSate(:,i+step)-nomSate(:,i-1+step)+dt*aircraft_dynamics(nomSate(:,i-1+step), nomControl(1,i-1+step), nomControl(2,i-1+step))-dt*Acurr(:,:,i-1+step)*nomSate(:,i-1+step)...
            -dt*Bcurr(:,:,i-1+step)*nomControl(:,i-1+step);
        xtraj(:,i)==Gnom+dt*Acurr(:,:,i-1+step)*xtraj(:,i-1)+xtraj(:,i-1)+ dt*Bcurr(:,:,i-1+step)*utraj(:,i-1);
        utraj(1,i-1)<=0.4363;
        utraj(1,i-1)>=-0.4363;
        utraj(2,i-1)<=pi/2;
        utraj(2,i-1)<=-pi/2;

    end
    cvx_end
    
    %[~,state] = ode45(@(t,S) aircraft_dynamics(S,utraj(1,1),utraj(2,1)),[0 control_dt],aircraft_state(:,step+1));
    %aircraft_state(:,step+2) = state(end,:);
    aircraft_state(:,step+2) = aircraft_state(:,step+1) + control_dt * aircraft_dynamics(aircraft_state(:,step+1),utraj(1,1),utraj(2,1));
    
end




%% plotting
figure;
plot3(nomSate(1,1:200),nomSate(2,1:200),nomSate(3,1:200),'m--');
hold on;
plot3(xtraj(1,:),xtraj(2,:),xtraj(3,:),'r');
plot3(aircraft_state(1,:),aircraft_state(2,:),aircraft_state(3,:),'b-.')
figure
hold on
plot(nomSate(4,1:200))
plot(aircraft_state(4,:))



