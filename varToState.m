function [x,y,h,V,gamma,chi,alpha,mu] = ...
    varToState(var)

global N

% Note that var = [x;y;h;V;gamma;chi;alpha;mu]
x = var(1:N+1);
y = var(N+2:2*N+2);
h = var(2*N+3:3*N+3);
V = var(3*N+4:4*N+4);
gamma = var(4*N+5:5*N+5);
chi = var(5*N+6:6*N+6);
alpha = var(6*N+7:7*N+6);
mu = var(7*N+7:8*N+6);

end