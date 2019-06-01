function B=B_jac(x,y,h,V,gamma,chi,alpha,mu)
%L,D,mu
dalpha=0.0001;
[dCL, dCD]=find_coeff(alpha+dalpha);
[CL, CD]=find_coeff(alpha);
dDalpha=(dCD-CD)/dalpha;
dLalpha=(dCL-CL)/dalpha;

g=9.81;
global W;
global g;
global Surface_area;
global rho_air;

qs=0.5*rho_air*V*V*Surface_area;
[CL CD]=find_coeff(alpha);
L=CL*qs;
D=CD*qs;




B1 = [0,0];
B2 = [0,0];
B3 = [0,0];
B4 = [-g/W*dDalpha,0];
B5 = [g*cos(mu)/(V*W)*dLalpha,-g/V*L/W*sin(mu)];
B6 = [dLalpha*g*sin(mu)/(V*cos(gamma)*W),g*cos(mu)/(V*cos(gamma)*W)*L];

B = [B1;B2;B3;B4;B5;B6];
end