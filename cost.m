% Cost of the problem

function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

% cost
c = (x(end)-5000)^2 + y(end)^2 + ...
    norm(alpha(2:end)-alpha(1:end-1))^2 + norm(mu(2:end)-mu(1:end-1))^2;