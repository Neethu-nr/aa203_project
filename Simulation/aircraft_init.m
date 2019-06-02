function aircraft_init()

% Global values
global W;W = 91200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad