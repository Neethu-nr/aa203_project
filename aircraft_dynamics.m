function dS = aircraft_dynamics(S, alpha, mu)
  
    global W;
    global Surface_area;
    global mu_g;
    global rE;
    global mE;
    global rho_air; 
    
    % Wind parameters
    global Vw;    
    global chiw;

    x = S(1); y = S(2); h = S(3);
    V = S(4);gamma = S(5);chi= S(6);
    g = mu_g*mE/((rE + h)*(rE + h));
    
    % Drag and Lift calculation
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
    
    dS =[xdot;  ydot;   hdot;
         Vdot; gammadot; chidot];
end