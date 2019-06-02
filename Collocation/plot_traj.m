function p = plot_traj(var)

[x,y,h,V,gamma,chi,alpha,mu] = varToState(var);

figure
hold on
plot3(x,y,h)

for i = 1:length(x)
   
    plot3([x(i) x(i)],[y(i) y(i)],[0 h(i)],'k')
    
end

axis([-2500 15000 -3000 3000 0 1500])

end