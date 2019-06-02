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
x0 = [0,0,1000,100,0.26,pi];

airplane_simulator(@aircraft_dynamics,0.01,u,x0,dt)
plot3(x,y,h,'g')