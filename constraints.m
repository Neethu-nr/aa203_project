% Function providing equality and inequality constraints
% ceq(var) = 0 and c(var) \le 0

function [c, ceq] = constraints(var)

global N;
global T;

global x0;
global y0;
global h0;
global V0;
global gamma0;
global chi0;
global alpha0;
global mu0;

global yf;
global hf;
global chif;

global Vmin;
global mulim;
global alphaMax;

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);
hsafe = 100;

% Put here constraint inequalities
c = [];

musafe = 5 / 180 * pi;
h_step = 1.0*T/(1.0*N);
h_l_speed = -4;
h_h_speed = 0.5;

for i = 1:N
    % Provide here dynamical constraints via the trapeziodal formula
    
    S = [x(i);y(i);h(i);V(i);gamma(i);chi(i)];
    Snext = [x(i+1);y(i+1);h(i+1);V(i+1);gamma(i+1);chi(i+1)];
    
    dS = aircraft_dynamics(S,alpha(i),mu(i));
    
    indices = length(S)*(i-1)+1:length(S)*i;
    
    ceq(indices) = Snext-S-h_step*dS;
    
    h_dot = dS(3);
    
   if h(i) <= hsafe 
       c = [c musafe-mu(i) mu(i)-musafe];
       c = [c h_dot-h_h_speed h_l_speed-h_dot];
   else
       c = [c 0 0];
       c = [c 0 0];
   end
    
end

% Put here initial and final conditions
ceq(1+6*N) = x(1) - x0;
ceq(2+6*N) = y(1) - y0;
ceq(3+6*N) = h(1) - h0;
ceq(4+6*N) = V(1) - V0;
ceq(8+6*N) = h(end) - hf;

end