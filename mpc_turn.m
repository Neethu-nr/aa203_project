clear all; close all;

mu = 6.67384e-11; 
W = 9200;  
dt=1;
g=9.81;
L=0.1;
D=0.1;
%% A matrix
A1=@(x,y,h,V,gamma,chi) dt*[-1/dt 0 0 cos(gamma)*cos(chi) -V*cos(chi)*sin(gamma) -V*cos(gamma)*sin(chi)];
A2=@(x,y,h,V,gamma,chi) dt*[0 -1/dt 0 cos(gamma)*sin(chi) -V*sin(gamma)*sin(chi) V*cos(gamma)*cos(chi)];
A3=@(x,y,h,V,gamma,chi) dt*[0 0 -1/dt sin(gamma) V*cos(gamma) 0];
A4=@(x,y,h,V,gamma,chi) dt*[0 0 0 -1/dt -g*cos(gamma) 0];
A5=@(x,y,h,V,gamma,chi) dt*[0 0 0 -g*((L/W)*cos(mu)-cos(gamma))/V^2 (g/V)*sin(gamma)-1/dt 0];
A6=@(x,y,h,V,gamma,chi) dt*[0 0 0 -g*(L*sin(mu)/W)/(V^2*cos(gamma)) (L*sin(mu)/W)*(-g/(V*(cos(gamma))^2*(-sin(gamma)))) 0-1/dt];
A=@(x,y,h,V,gamma,chi) [A1(x,y,h,V,gamma,chi);A2(x,y,h,V,gamma,chi)...
    ;A3(x,y,h,V,gamma,chi);A4(x,y,h,V,gamma,chi);A5(x,y,h,V,gamma,chi)...
    ;A6(x,y,h,V,gamma,chi)];

%% B matrix
B=@(L,D,V,gamma) dt*[0 0; 0 0;0 0;0 -g/W; g*cos(mu)/(V*W) 0;g*sin(mu)/(V*cos(gamma)*W) 0];

%% initial conditions
x0 = 0; y0 = 0; h0 = 100;
V0 = 100; % m/s
gamma0 = 0.26; % radians
chi0 = 0; % radians

%% test
curr_control=[L;D];
curr_state=[x0;y0;h0;V0;gamma0;chi0];
for i=1:100
    newstate(:,i)=A(curr_state(1),curr_state(2),curr_state(3),curr_state(4),...
        curr_state(5),curr_state(6))*curr_state+B(curr_control(1),curr_control(2),curr_state(4),curr_state(5))*curr_control;
    curr_state=newstate(:,i);
end
 figure;plot(newstate(1,:), newstate(2,:));
