function A = A_jac(x,y,h,V,gamma,chi,alpha,mu)

    global W;
    global Surface_area;
    global rho_air; 
    global mu_g;
    global rE;
    global mE;
    
    g = mu_g*mE/((rE + h)*(rE + h));
    dgdh = -2*mu_g*mE/(rE + h)^3;
    
    % Drag and Lift calculation
    qS = 0.5 * rho_air * V*V * Surface_area;
    [CL, CD] = find_coeff(alpha);
    L = CL * qS;
    D = CD * qS;
    
    dLdV = CL * rho_air *V * Surface_area;
    dDdV = CD * rho_air *V * Surface_area;

    A1=[0 0 0 cos(gamma)*cos(chi) -V*cos(chi)*sin(gamma) -V*cos(gamma)*sin(chi)];
    
    A2=[0 0 0 cos(gamma)*sin(chi) -V*sin(gamma)*sin(chi) V*cos(gamma)*cos(chi)];
    
    A3=[0 0 0 sin(gamma) V*cos(gamma) 0];
    
    A4=[0 0 dgdh/W*(-D-W*sin(gamma)) -g/W*dDdV -g*cos(gamma) 0];
    
    A5=[0 0 dgdh/V*(L/W*cos(mu)-cos(gamma)) g/V*dLdV/W*cos(mu)-g*(L/W*cos(mu)-cos(gamma))/V^2 g/V*sin(gamma) 0];
    
    A6=[0 0 dgdh*L/V/W*sin(mu)/cos(gamma) g*dLdV/W*sin(mu)/(V*cos(gamma))-g*L/W*sin(mu)/(V^2*cos(gamma)) g*L/W/V*sin(mu)/cos(gamma)^2*sin(gamma) 0];

    A = [A1;A2;A3;A4;A5;A6];

end
