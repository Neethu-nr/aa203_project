function c = cost(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

%Minimize changes in control
u = [alpha';mu'];
du = u(:,2:end) - u(:,1:end-1);
du = sum(norms(du,2));

%c = -x(end) + du;
c = -x(end);