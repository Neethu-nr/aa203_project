% Function providing equality and inequality constraints
% ceq(var) = 0 and c(var) \le 0

function [c,ceq] = constraint(var)

global N;
global T;

global x0;
global y0;
global h0;
global V0;
global gamma0;
global chi0;

global yf;
global hf;
global chif;

global Vmin;
global mulim;
global alphaMax;

[x,y,h,V,gamma,chi,alpha,mu] = varToState(varInit);


% Put here constraint inequalities
c = [-alpha;alpha-alphaMax;mulim-mu;mu-mulim;Vmin-V(end)];

% Computing dynamical constraints via the trapezoidal rule
h_step = 1.0*T/(1.0*N);
for i = 1:N
    % Provide here dynamical constraints via the trapeziodal formula
    
    S = [x(i);y(i);h(i);V(i);gamma(i);chi(i)];
    Snext = [x(i+1);y(i+1);h(i+1);V(i+1);gamma(i+1);chi(i+1)];
    
    dS = aircraft_dynamics(S,alpha(i),mu(i));
    dSnext = aircraft_dynamics(Snext,alpha(i+1),mu(i+1));
    
    indices = length(S)*(i-1)+1:length(S)*i;
    
    ceq(indices) = Snext-S-h_step*(dS+dSnext)/2;
end

% Put here initial and final conditions
ceq(1+6*N) = x(1) - x0;
ceq(2+6*N) = y(1) - y0;
ceq(3+6*N) = h(1) - h0;
ceq(4+6*N) = V(1) - V0;
ceq(5+6*N) = gamma(1) - gamma0;
ceq(6+6*N) = chi(1) - chi0;
ceq(7+6*N) = y(end) - yf;
ceq(8+6*N) = h(end) - hf;
ceq(9+6*N) = chi(end) - chif;