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
Acurr = zeros(6,6,length(t_desired));
Bcurr = zeros(6,2,length(t_desired));

for i=1:length(t_desired)
    Acurr(:,:,i)=A_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    Bcurr(:,:,i)=B_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
end

dt=0.01;
mpc_horizon=100;
mpc_steps = 1;

aircraft_state = zeros(6,mpc_steps+1);
aircraft_state(:,1) = nomSate(:,1);
control_dt = 0.01;

Q = diag([1 1 1 1 1 1]);
Qhalf = sqrtm(Q);

for step = 0:mpc_steps-1
    
    disp(step+1);
    
    stateCost=0;
    controlCost=0;

    cvx_quiet True
    cvx_begin
    variables xtraj(6,mpc_horizon) utraj(2,mpc_horizon-1)
    for i=1:mpc_horizon
        
        idx = i+step;

        stateCost=stateCost+norm(Qhalf*(xtraj(:,i)-nomSate(:,idx)),'fro');

    end
    minimize stateCost+controlCost
    
    subject to
    
        xtraj(:,1)==aircraft_state(:,1+step)

        for i=1:mpc_horizon-1

            idx = i+step;
            
            x_bar = nomSate(:,idx);
            x_bar_1 = nomSate(:,idx+1);
            u_bar = nomControl(:,idx);
            f_bar = aircraft_dynamics(x_bar,u_bar(1),u_bar(2));
            A_bar = Acurr(:,:,idx);
            B_bar = Bcurr(:,:,idx);
            G_bar=x_bar_1-x_bar+dt*(f_bar-A_bar*x_bar-B_bar*u_bar);
            
            xtraj(:,i+1)==xtraj(:,i)+dt*((A_bar)*xtraj(:,i)+B_bar*utraj(:,i)+G_bar);
            utraj(1,i)<=0.4363;
            utraj(1,i)>=-0.4363;
            utraj(2,i)<=pi/2;
            utraj(2,i)<=-pi/2;

        end
        
    cvx_end
    
    [~,state] = ode45(@(t,S) aircraft_dynamics(S,utraj(1,1),utraj(2,1)),[0 control_dt],aircraft_state(:,step+1));
    aircraft_state(:,step+2) = state(end,:);

end




% plotting
figure;
hold on;
plot3(nomSate(1,1:200),nomSate(2,1:200),nomSate(3,1:200),'m--');
plot3(xtraj(1,:),xtraj(2,:),xtraj(3,:),'r');
plot3(aircraft_state(1,:),aircraft_state(2,:),aircraft_state(3,:),'b-.')