function B=B_jac(x,y,h,V,gamma,chi,alpha,mu)

    % --- Theoretical derivative ---
%     global mu_g;
%     global rE;
%     global mE;
%     global W;
%     global Surface_area;
%     
%     g = mu_g*mE/((rE + h)*(rE + h));
%     
%     %L,D,mu
%     dalpha=1e-4;
%     [dCL, dCD]=find_coeff(alpha+dalpha);
%     [CL, CD]=find_coeff(alpha);
%     dDalpha=(dCD-CD)/dalpha;
%     dLalpha=(dCL-CL)/dalpha;
%     
%     rho_air = rho_air_calc(h);
%     qS = 0.5 * rho_air * V*V * Surface_area;
%     [CL, CD] = find_coeff(alpha);
%     L = CL * qS;
%     
%     B1 = [0,0];
%     B2 = [0,0];
%     B3 = [0,0];
%     B4 = [-g/W*dDalpha,0];
%     B5 = [g*cos(mu)/(V*W)*dLalpha,-g/V*L/W*sin(mu)];
%     B6 = [dLalpha*g*sin(mu)/(V*cos(gamma)*W),g*cos(mu)/(V*cos(gamma)*W)*L];
%     
%     B_jac = [B1;B2;B3;B4;B5;B6]

    % --- Numerical derivative ---
    dt = 1e-8;
    S = [x,y,h,V,gamma,chi]';
    B1 = (aircraft_dynamics(S,alpha+dt,mu)-aircraft_dynamics(S,alpha-dt,mu))/(2*dt);

    B2 = (aircraft_dynamics(S,alpha,mu+dt)-aircraft_dynamics(S,alpha,mu-dt))/(2*dt);
    
    B = [B1 B2];

end