function dS = aircraft_dynamics(S, u)
  
    global W;
    global Surface_area;
    global mu_g;
    global rE;
    global mE;
    
    % Wind parameters
    global Vw;    
    global chiw;
    
    alpha = u(1);
    mu = u(2);

    x = S(1); y = S(2); h = S(3);
    V = S(4);gamma = S(5);chi= S(6);
    g = mu_g*mE/((rE + h)*(rE + h));
    
    % Drag and Lift calculation
    rho_air = rho_air_calc(h);
    qS = 0.5 * rho_air * V*V * Surface_area;
    [CL, CD] = find_coeff(alpha);
    L = CL * qS;
    D = CD * qS;
    
    % Derivative calculation
    xdot = V*cos(gamma)*cos(chi) + Vw*cos(chiw);
    ydot = V*cos(gamma)*sin(chi) + Vw*sin(chiw);
    hdot = V*sin(gamma);
    Vdot = -g/W*(D + W*sin(gamma));
    gammadot = g/V*(L/W*cos(mu) - cos(gamma));
    chidot = g*L*sin(mu) / (V*cos(gamma)*W);
    
    dS = [xdot;  ydot; hdot; Vdot; gammadot; chidot];
end