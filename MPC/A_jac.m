function A = A_jac(x,y,h,V,gamma,chi,alpha,mu)

% --- Theoretical derivative ---

    global W;
    global Surface_area;
    global mu_g;
    global rE;
    global mE;
    
    g = mu_g*mE/((rE + h)*(rE + h));
    
    % Drag and Lift calculation
    rho_air = rho_air_calc(h);
    qS = 0.5 * rho_air * V*V * Surface_area;
    [CL, CD] = find_coeff(alpha);
    L = CL * qS;
    D = CD * qS;
    
    dt = 1e-4;
    dgdh = -2*mu_g*mE/(rE + h)^3;
    drhodh = (rho_air_calc(h+dt) - rho_air_calc(h))/dt;
    
    dLdV = CL * rho_air *V * Surface_area;
    dDdV = CD * rho_air *V * Surface_area;
    
    dLdh = CL * 0.5 * drhodh * V*V * Surface_area;
    dDdh = CD * 0.5 * drhodh * V*V * Surface_area;
    
    A11 = 0; A12 = 0; A13 = 0;
    A14 = cos(gamma)*cos(chi);
    A15 = -V*sin(gamma)*cos(chi);
    A16 = -V*cos(gamma)*sin(chi);
    
    A21 = 0; A22 = 0; A23 = 0;
    A24 = cos(gamma)*sin(chi);
    A25 = -V*sin(gamma)*sin(chi);
    A26 = V*cos(gamma)*cos(chi);
    
    A31 = 0; A32 = 0; A33 = 0;
    A34 = sin(gamma);
    A35 = V*cos(gamma);
    A36 = 0;
    
    A41 = 0; A42 = 0;
    A43 = -dgdh/W*(D+W*sin(gamma))-dDdh*g/W;
    A44 = -g*dDdV/W;
    A45 = -g*cos(gamma);
    A46 = 0;
    
    A51 = 0; A52 = 0;
    A53 = dgdh/V*(L/W*cos(mu)-cos(gamma))+g*dLdh/V/W*cos(mu);
    A54 = -g/V^2*(L/W*cos(mu)-cos(gamma))+g/V*dLdV/W*cos(mu);
    A55 = g/V*sin(gamma);
    A56 = 0;
    
    A61 = 0; A62 = 0;
    A63 = sin(mu)/(V*cos(gamma)*W)*(dgdh*L+g*dLdh);
    A64 = g/cos(gamma)*sin(mu)/W*(dLdV/V-L/V^2);
    A65 = g/V*L/W*sin(mu)*sin(gamma)/cos(gamma)^2;
    A66 = 0;
    
    A = [A11 A12 A13 A14 A15 A16;
         A21 A22 A23 A24 A25 A26;
         A31 A32 A33 A34 A35 A36;
         A41 A42 A43 A44 A45 A46;
         A51 A52 A53 A54 A55 A56;
         A61 A62 A63 A64 A65 A66];

%--- Numerical derivative ---
%     dt = 1e-8;
%     S = [x,y,h,V,gamma,chi]';
%     dS = [x+dt,y,h,V,gamma,chi]';
%     A1 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
% 
%     dS = [x,y+dt,h,V,gamma,chi]';
%     A2 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
%     
%     dS = [x,y,h+dt,V,gamma,chi]';
%     A3 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
%     
%     dS = [x,y,h,V+dt,gamma,chi]';
%     A4 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
%     
%     dS = [x,y,h,V,gamma+dt,chi]';
%     A5 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
%     
%     dS = [x,y,h,V,gamma,chi+dt]';
%     A6 = (aircraft_dynamics(dS,alpha,mu)-aircraft_dynamics(S,alpha,mu))/dt;
%     
%     A = [A1 A2 A3 A4 A5 A6]

end
