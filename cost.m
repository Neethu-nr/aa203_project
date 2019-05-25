% Cost of the problem

function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

% cost
dalpha = alpha(2:end) - alpha(1:end-1);
dmu = mu(2:end) - mu(1:end-1);
du = (norm(dalpha)^2 + norm(dmu)^2);

S = [x';y';h';V';gamma';chi'];
dS = S(:,2:end) - S(:,1:end-1);
dS = sum(sum(dS.^2));

lambda = 1;

c = x(end) + dS ;