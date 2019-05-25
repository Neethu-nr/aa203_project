% Cost of the problem

function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

% cost
c = (x(end)-4200)^2 + (y(end)-1900)^2;