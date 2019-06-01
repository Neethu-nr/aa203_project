function yref = get_reference_trajectory(t)
T = 155;
N = 40;
global reference_traj; 
yref = reference_traj(end,:);
if t< T
    time = 0:T/N:T;
    yref(1) = spline(time,reference_traj(:,1),t);
    yref(2) = spline(time,reference_traj(:,2),t);
    yref(3) = spline(time,reference_traj(:,3),t);
    yref(4) = spline(time,reference_traj(:,4),t);
    yref(5) = spline(time,reference_traj(:,5),t);
    yref(6) = spline(time,reference_traj(:,6),t);
end