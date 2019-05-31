function B_jac=B_jac(x,y,h,V,gamma,chi,alpha,mu)
%L,D,mu
dalpha=0.0001;
[dCL, dCD]=find_coeff(alpha+dalpha);
[CL, CD]=find_coeff(alpha);
dDalpha=(dCD-CD)/dalpha;
dLalpha=(dCL-CL)/dalpha;

B1 = [0,0];
B1 = [0,0];
B1 = [0,0];
B4 = [-g/W*dDalpha,0];
B5 = [g*cos(mu)/(V*W)*dLalpha,-g/V*L/W*sin(mu)];
B6 = [dLalpha*g*sin(mu)/(V*cos(gamma)*W),g*cos(mu)/(V*cos(gamma)*W)*L];
end