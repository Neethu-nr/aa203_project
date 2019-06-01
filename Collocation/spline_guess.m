function guess = spline_guess(x0,xf,m,N,plot_status)

    dist = norm(x0(1:2)-xf(1:2));
    offset = (x0(1) + xf(1)) / 2;
    
    dt = 0.01;
    t = pi:-dt:dt;

    x = dist / 2 * cos(t) - offset - 0.2 * dist * sin(t);
    y = dist /2 * sin(t).*sin(1/2*t).^m;
    z = linspace(x0(3),xf(3),length(t));
    
    dx = x(2:end) - x(1:end-1);
    dy = y(2:end) - y(1:end-1);
    
    avg_len = sum(norms([dx;dy])) / N;
    
    len = 0;
    
    x_sample = zeros(1,N+1);
    y_sample = zeros(1,N+1);
    z_sample = zeros(1,N+1);
    
    count = 1;
    
    x_sample(count) = x(1);
    y_sample(count) = y(1);
    z_sample(count) = z(1);
    
    for i = 1:length(t)-1
        
        len = len + norm([dx(i);dy(i)]);
        
        if len > count * avg_len
           
            count = count + 1;
            x_sample(count) = x(i);
            y_sample(count) = y(i);
            z_sample(count) = z(i);
            
        end
        
    end
    
    x_sample(end) = x(end);
    y_sample(end) = z(end);
    z_sample(end) = z(end);
    
    if plot_status
    
        hold on
        plot3(x,y,z,'r')
        plot3(x_sample,y_sample,z_sample,'.')
        axis equal
    
    end
    
    Vmin = 100;
    V = Vmin * ones(1,N+1);
    dx = x_sample(2:end) - x_sample(1:N);
    dy = y_sample(2:end) - y_sample(1:N);
    dz = z_sample(2:end) - z_sample(1:N);
    
    gamma = [atan2(dz, norms([dx;dy])) 0];
    
    chi = [atan2(dy, dx) 0];
    
    guess = [x_sample';y_sample';z_sample';V';gamma';chi'];

end