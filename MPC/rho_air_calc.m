function rho = rho_air_calc(h)

    height = [-1000 0 1000 2000 3000 4000 5000];
    rho_air = [1.347 1.225 1.112 1.007 0.9093 0.8194 0.7364];

    rho = spline(height,rho_air,h);
    
end