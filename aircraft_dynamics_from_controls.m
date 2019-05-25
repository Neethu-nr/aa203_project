% dynamics function  for when we have control for all time 
function dS = aircraft_dynamics_from_controls(t,S, alpha, mu,time_table)
    alpha_t = spline(time_table, alpha, t);
    mu_t = spline(time_table, mu, t);
    dS = aircraft_dynamics(S, alpha_t, mu_t);
end

