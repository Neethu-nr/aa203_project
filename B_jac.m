function B_jac=B_jac(x,y,h,V,gamma,chi,alpha,mu)

global g;
global W;
global Surface_area;
global rho_air; 

%L,D,mu
dalpha=1e-10;
[dCL, dCD]=find_coeff(alpha+dalpha);
[CL, CD]=find_coeff(alpha);
dDalpha=(dCD-CD)/dalpha;
dLalpha=(dCL-CL)/dalpha;

qS = 0.5 * rho_air * V*V * Surface_area;
[CL, CD] = find_coeff(alpha);
L = CL * qS;
D = CD * qS;

B1 = [0,0];
B2 = [0,0];
B3 = [0,0];
B4 = [-g/W*dDalpha,0];
B5 = [g*cos(mu)/(V*W)*dLalpha,-g/V*L/W*sin(mu)];
B6 = [dLalpha*g*sin(mu)/(V*cos(gamma)*W),g*cos(mu)/(V*cos(gamma)*W)*L];

B_jac = [B1;B2;B3;B4;B5;B6];
end