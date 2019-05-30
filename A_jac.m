function A = A_jac(x,y,h,V,gamma,chi,alpha,mu)

    global W;
    global Surface_area;
    global mu_g;
    global rE;
    global mE;
    global rho_air; 

    g = mu_g*mE/((rE + h)*(rE + h));
    
    % Drag and Lift calculation
    qS = 0.5 * rho_air * V*V * Surface_area;
    [CL, ~] = find_coeff(alpha);
    L = CL * qS;

    A1=[0 0 0 cos(gamma)*cos(chi) -V*cos(chi)*sin(gamma) -V*cos(gamma)*sin(chi)];
    A2=[0 0 0 cos(gamma)*sin(chi) -V*sin(gamma)*sin(chi) V*cos(gamma)*cos(chi)];
    A3=[0 0 0 sin(gamma) V*cos(gamma) 0];
    A4=[0 0 0 0 -g*cos(gamma) 0];
    A5=[0 0 0 -g*(L/W*cos(mu)-cos(gamma))/V^2 g/V*sin(gamma) 0];
    A6=g*L/W*[0 0 0 -sin(mu)/(V^2*cos(gamma)) V*sin(mu)*cos(gamma)^2*sin(gamma) 0];

    A = [A1;A2;A3;A4;A5;A6];

end
