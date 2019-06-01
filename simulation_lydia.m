close all;

% Global values
global W;W = 9200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg
global rho_air;rho_air = 1.225 ;        % density of air in kg/m3

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

% Control values
alpha = 0; % angle of attack in radians
mu = 0.1;   % rolling angle in radians
dt = 4;

x0 = 0; y0 = 0; h0 = 8000;
V0 = 100; % m/s
gamma0 = 0; % radians
chi0 = 0; % radians
S0 = [x0 ; y0; h0;
    V0; gamma0; chi0];

% Nominal controls and trajectory
[xnom,ynom,hnom,Vnom,gammanom,chinom,alphanom,munom]=varToState(var);


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
figure;
plot3(xnom,ynom,hnom);
nomSate=[xnom';ynom';hnom';Vnom';gammanom';chinom'];
nomControl=[alphanom';munom'];


dt=0.1;
iterations=100;
% define all A and B matrices
for i=1:iterations
    Acurr(:,:,i)=A_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    Bcurr(:,:,i)=B_jac(xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i),alphanom(i),munom(i));
    
end

stateCost=[];
controlCost=[];

cvx_begin
variables xtraj(6,iterations) utraj(2,iterations-1)
for i=1:iterations
    weight=zeros(6,6);
    weight(3,3)=3;
    weight=weight+eye(6);
    stateCost=stateCost+norm(eye(6)*(xtraj(:,i)-nomSate(:,i)));
    if i>=2
    controlCost=stateCost+norm(eye(2)*(utraj(:,i-1)-nomControl(:,i-1)));
    end
end
minimize stateCost+controlCost
%initial conditions
xtraj(:,1)==[x0 ; y0; h0;
    V0; gamma0; chi0];
S=[xnom(i),ynom(i),hnom(i),Vnom(i),gammanom(i),chinom(i)];
U=[alphanom(i), munom(i)];
for i=2:iterations
    Gnom=nomSate(:,i)-nomSate(:,i-1)+dt*aircraft_dynamics(nomSate(:,i-1), nomControl(1,i-1), nomControl(2,i-1))-dt*Acurr(:,:,i-1)*nomSate(:,i-1)...
        -dt*Bcurr(:,:,i-1)*nomControl(:,i-1);
    xtraj(:,i)==Gnom+dt*Acurr(:,:,i-1)*xtraj(:,i-1)+xtraj(:,i-1)+ dt*Bcurr(:,:,i-1)*utraj(:,i-1);
    utraj(1,i-1)<=0.4363;
    utraj(1,i-1)>=-0.4363;
    utraj(2,i-1)<=1.3;
    utraj(2,i-1)<=-1.3;
    (xtraj(3,i-1)-xtraj(3,i))<500;
    
end
cvx_end



%% plotting
figure;plot3(xtraj(1,:),xtraj(2,:),xtraj(3,:));
figure;plot3(nomSate(1,:),nomSate(2,:),nomSate(3,:));


