% Cost of the problem

function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(varInit);

% cost
c = x(end);