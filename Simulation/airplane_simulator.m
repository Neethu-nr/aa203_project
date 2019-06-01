function airplane_simulator(dyn,time,u,x0)

    x = zeros(length(x0),length(time));
    x(:,1) = x0;

    for k = 1:length(time)-1
       
        [~, x(:,k+1)] = ode45(@(t,x) dyn(x,u(k),[]))
        
        
        
    end

end