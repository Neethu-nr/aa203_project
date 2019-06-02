function [c, ceq] = constraints(var)

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

%State varialbes
[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

%Initialize variables
hsafe = 100;

h_step = 1.0*T/(1.0*N);
h_l_speed = -4;
h_h_speed = 0.5;

Snext = [x(1);y(1);h(1);V(1);gamma(1);chi(1)];
dSnext = aircraft_dynamics(Snext,alpha(1),mu(1));

ceq = zeros(11+6*N,1);
c = zeros(3*(N+1),1);

for i = 1:N
    
    % ---trapeziodal constraints----
    
%     S = Snext;
%     Snext = [x(i+1);y(i+1);h(i+1);V(i+1);gamma(i+1);chi(i+1)];
%     
%     dS = dSnext;
%     dSnext = aircraft_dynamics(Snext,alpha(i+1),mu(i+1));
%     
%     indices = length(S)*(i-1)+1:length(S)*i;
%     
%     ceq(indices) = Snext-S-h_step*(dS + dSnext)/2;
    

    % ---Hermite-Simpson constraints----
    S = Snext;
    Snext = [x(i+1);y(i+1);h(i+1);V(i+1);gamma(i+1);chi(i+1)];
    
    dS = dSnext;
    dSnext = aircraft_dynamics(Snext,alpha(i+1),mu(i+1));  
    
    alpha_c = (alpha(i) + alpha(i+1)) / 2;
    mu_c = (mu(i) + mu(i+1)) / 2;
    S_c = 1/2 * (S + Snext) + h_step/8*(dS-dSnext);
    
    dSc = aircraft_dynamics(S_c,alpha_c,mu_c);
    
    indices = length(S)*(i-1)+1:length(S)*i;
    
    ceq(indices) = Snext-S-h_step/6*(dS+4*dSc+dSnext);

% Normal constraints

%     S = [x(i);y(i);h(i);V(i);gamma(i);chi(i)];
%     Snext = [x(i+1);y(i+1);h(i+1);V(i+1);gamma(i+1);chi(i+1)];
% 
%     dS = aircraft_dynamics(S,alpha(i),mu(i));
%     
%     indices = length(S)*(i-1)+1:length(S)*i;
%     
%     ceq(indices) = Snext - S - h_step*dS;
    

    % Special constraints on low height 
    h_dot = dS(3);
    
   if h(i) <= hsafe 
       c1 = [h_dot-h_h_speed h_l_speed-h_dot];
       c2 = Vmin-V(i);
       indeces = 3*(i-1)+1:3*i;
       c(indeces) = [c1 c2];
   end
   
   if i == N && h(i+1) <= hsafe
     h_dot_next = dSnext(3);
     c1 = [h_dot_next-h_h_speed h_l_speed-h_dot_next];
     c2 = Vmin-V(i+1);
     indeces = 3*(i)+1:3*(i+1);
     c(indeces) = [c1 c2];
       
   end
    
end

% Initial and final conditions
ceq(1+6*N) = x(1) - x0;
ceq(2+6*N) = y(1) - y0;
ceq(3+6*N) = h(1) - h0;
ceq(4+6*N) = V(1) - V0;
ceq(5+6*N) = chi(1) - chi0;
ceq(6+6*N) = gamma(1) - gamma0;
ceq(7+6*N) = y(end) - yf;
ceq(8+6*N) = h(end) - hf;
ceq(9+6*N) = chi(end) - chif;

end