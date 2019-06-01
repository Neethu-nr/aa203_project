function final_state = plane(S0,mv,Ts)
   tspan = [0,Ts];
   S0;
   mv;
   [~,state] = ode45(@(t,S) aircraft_dynamics(S,mv(1),mv(2)),tspan,S0); 
   final_state = state(end,:)';
end