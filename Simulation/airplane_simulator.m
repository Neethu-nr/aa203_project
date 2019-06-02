function airplane_simulator(dyn,dt,u,x0,udt)

    global T;
    
    sample_t = 0:udt:T;
    sim_t = 0:dt:T;
    x = zeros(length(x0),length(sim_t));
    x(:,1) = x0;

    for k = 1:length(sim_t)-1
        
        u1 = spline(sample_t,u(1,:),sim_t(k));
        u2 = spline(sample_t,u(2,:),sim_t(k));
        
        x(:,k+1) = x(:,k) + dt * dyn(x(:,k),[u1;u2]);
        
    end
    
    figure;

    hold on
    plot3(x(1,:),x(2,:),x(3,:))
    
    d = floor(length(x) / 100);

    for i = 1:d:length(x)
   
        plot3([x(1,i) x(1,i)],[x(2,i) x(2,i)],[0 x(3,i)],'k')
    
    end

end