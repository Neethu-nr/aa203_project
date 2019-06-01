% Cost of the problem

function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

% cost
%dalpha = alpha(2:end) - alpha(1:end-1);
%dmu = mu(2:end) - mu(1:end-1);
%du = (norm(dalpha)^2 + norm(dmu)^2);

xyz = [x';y'];

dxyz = xyz(:,2:end) - xyz(:,1:end-1);
dxyz = sum(norms(dxyz,2));

c = -x(end) + dxyz ;