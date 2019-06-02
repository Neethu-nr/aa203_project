close all
clear
clc

aircraft_init()

res = matfile('res.mat');
global N;N=res.N;
global T;T=res.T;
dt = T/N;

[x,y,h,V,gamma,chi,alpha,mu] = varToState(res.var);

u = [alpha';mu'];
x0 = [x(1) y(1) h(1) V(1) gamma(1) chi(1)];

airplane_simulator(@aircraft_dynamics,0.01,u,x0,dt)
plot3(x,y,h,'g')