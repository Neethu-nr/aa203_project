close all;
clear
clc

% Global values
global W;W = 91200;                % Weight of aircraft in kg
global Surface_area;Surface_area =27.87;                % Surface area of wings in m.sq       % density of air in kg/m3
global mu_g;mu_g = 6.67384e-11;      % gravitational constant in m.3kg.-1s.-2
global rE;rE = 6.3781*1e6;             %in radius of earth in m
global mE;mE = 5.972*1e24;             %mass earth in kg

% Wind parameters
global Vw;Vw = 0;    % Magnitude of wind in m/s
global chiw;chiw = 0;  % Direction of wind in rad

res = matfile('nom_traj.mat');
Acurr = res.Acurr;
Bcurr = res.Bcurr;
nomState = res.nomState;
nomControl = res.nomControl;
t_desired = res.t_desired;
dt = res.dt;

mpc_horizon=10;
mpc_steps = 1720;
start = 0;

aircraft_state = zeros(6,mpc_steps+1);
aircraft_control = zeros(2,mpc_steps);
aircraft_state(:,1) = nomState(:,1+start);
control_dt = 0.1;

aircraft_state_no_mpc = zeros(6,mpc_steps+1);
aircraft_state_no_mpc(:,1) = nomState(:,1+start);

Q = diag([1 1 1 1 1 1]);
Qhalf = sqrtm(Q);
R = diag([100 300]);
Rhalf = sqrtm(R);

time = 0;

for step = start:mpc_steps-1+start
    
    stateCost=0;
    controlCost=0;
    
    tic;

    cvx_quiet True
    cvx_begin
    variables xtraj(6,mpc_horizon) utraj(2,mpc_horizon-1)
    for i=1:mpc_horizon
        
        idx = i+round(control_dt/dt)*step;
        stateCost=stateCost+pow_pos(norm(Qhalf*(xtraj(:,i)-nomState(:,idx)),'fro'),2);
        
        if i < mpc_horizon
            controlCost=controlCost+pow_pos(norm(Rhalf*(utraj(:,i)-nomControl(:,idx)),'fro'),2);
        end

    end
    minimize stateCost+controlCost
    
    subject to
    
        xtraj(:,1)==aircraft_state(:,1+step-start)
        xtraj(3,:) >= 0

    for i=1:mpc_horizon-1
        idx = i+round(control_dt/dt)*step;
        x_bar_1 = nomState(:,idx+1);
        x_bar = nomState(:,idx);
        u_bar = nomControl(:,idx);
        f_bar = aircraft_dynamics(x_bar,u_bar(1),u_bar(2));
        A_bar = Acurr(:,:,idx);
        B_bar = Bcurr(:,:,idx);
        G_bar=x_bar_1-x_bar+dt*(f_bar-A_bar*x_bar-B_bar*u_bar);
        
        xtraj(:,i+1)==G_bar+dt*A_bar*xtraj(:,i)+xtraj(:,i)+ dt*B_bar*utraj(:,i);
        utraj(1,i)<=0.4363;
        utraj(1,i)>=-0.4363;
        utraj(2,i)<=pi/2;
        utraj(2,i)>=-pi/2;

    end
        
    cvx_end
    
    sim_time = 0:dt:control_dt;
    state = zeros(size(aircraft_state,1),length(sim_time));
    state(:,1) = aircraft_state(:,step+1-start);
    
    %Add noisey wind
    Vw = 10*rand();
    
    aircraft_state(:,step+2-start) = aircraft_state(:,step+1-start)+...
        dt*aircraft_dynamics(aircraft_state(:,step+1-start),utraj(1,1),utraj(2,1));
    aircraft_state_no_mpc(:,step+2-start) = aircraft_state_no_mpc(:,step+1-start)+...
        dt*aircraft_dynamics(aircraft_state_no_mpc(:,step+1-start),nomControl(1,step+1-start),nomControl(2,step+1-start));
  
    Vw = 0;
    chiw = 0;
    
    time = time + toc;
    fprintf("Iteration %d, average time: %.1f, time left: %.1f.\n",step+1,time/(step+1-start),(mpc_steps/(step+1-start)-1)*time)

end


save('mpc_res','aircraft_state','aircraft_state_no_mpc','nomState');

%% plotting
close all

figure;
hold on;
plot3(nomState(1,:),nomState(2,:),nomState(3,:),'c','linewidth',1.5);
plot3(aircraft_state_no_mpc(1,:),aircraft_state_no_mpc(2,:),aircraft_state_no_mpc(3,:),'r--','linewidth',1.3);
plot3(aircraft_state(1,:),aircraft_state(2,:),aircraft_state(3,:),'k--','linewidth',1.3)
title('MPC tracking with noise (random wind speed)','fontsize',18)

for i = round(linspace(1,length(aircraft_state),40))
   
    %plot3([nomState(1,i) nomState(1,i)],[nomState(2,i) nomState(2,i)],[0 nomState(3,i)],'g')
    %plot3([aircraft_state_no_mpc(1,i) aircraft_state_no_mpc(1,i)],[aircraft_state_no_mpc(2,i) aircraft_state_no_mpc(2,i)],[0 aircraft_state_no_mpc(3,i)],'r')
    plot3([aircraft_state(1,i) aircraft_state(1,i)],[aircraft_state(2,i) aircraft_state(2,i)],[0 aircraft_state(3,i)],'k')
    
end
legend('Nominal trajectory','Without MPC', 'MPC','fontsize',16)
xlabel('x','fontsize',16)
ylabel('y','fontsize',16)
zlabel('h','fontsize',16)


