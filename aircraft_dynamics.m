function dS = aircraft_dynamics(S)
  
    global W = 9200; % in kg
    global S =27.87; % in m.sq
    global mu_g;
    global rE;
    global rho_air; %density of air
    g = mu_g/((rE + y)*(rE + y)); % gravitational constant
    
    % Wind parameters
    global Vw = 0;    
    global chiw = 0;

    x = S(1); y = S(2); h = S(3);
    V = S(4);gamma = S(5);chi= S(6);
    
    % Drag and Lift calculation
    qS = 0.5 * rho_air*V*V*S;
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