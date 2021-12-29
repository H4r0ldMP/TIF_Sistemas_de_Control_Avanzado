function dx = BSMCF(t,x)% t,x entradas y dx=x1dot,x2dot salidas
c1 = 15; c2 = 12; k = 10; L = 4; v1 = 40; rho = 0.001;
m = 1525;  a = 1.1; b = 1.67; Ca = 67; Cb = 67; Iz = 2305;
%val_des = 0.25;
%val_des1 = 0;
%val_des2 = 0;
val_des = sin(t);
val_des1 = cos(t);
val_des2 = -sin(t);
%s1 = (thd-x(2))+c*(th-x(1));
%z1 = val_des-x(1); 
%z1d = x(2)-val_des1;
z1 = -val_des+x(1); 
z1d = -val_des1+x(2);
s1 = val_des1-x(2)+c1*z1;
g = (-2*L*((a*Ca + b*Cb)/(Iz*v1)) - (2*(Ca+Cb)/(m*v1)));
H = (-v1-(2*(a*Ca-b*Cb)/(m*v1)) + L*2*((a*a*Ca+b*b*Cb)/(Iz*v1)))*rho*v1;
d = ((2*Ca)/m) + ((2*a*Ca)/Iz);
u = -1/d*(z1+g*(x(1))+H+c1*z1d+c2*s1-val_des2)-k*sign(s1); 

x1dot = x(2);
x2dot = g*x(1)+d*u+H;
dx = [x1dot;x2dot];
end


