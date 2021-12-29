clc;clear all; close all;
[t,x] = ode45(@BSMCF,[0 20],[-2 -2]);
u = zeros(length(t),1);
c1 = 15; c2 = 12; k = 10; L = 4; v1 = 40; rho = 0.001;
m = 1525;  a = 1.1; b = 1.67; Ca = 67; Cb = 67; Iz = 2305;
for i=1:length(t)
%val_des = 0.25;
%val_des1 = 0;
%val_des2 = 0;
val_des = sin(t(i));
val_des1 = cos(t(i));
val_des2 = -sin(t(i));
%z1 = val_des-x(i,1); 
%z1d = x(i,2)-val_des1;
z1 = -val_des+x(i,1); 
z1d = -val_des1+x(i,2);
g = (-2*L*((a*Ca + b*Cb)/(Iz*v1)) - (2*(Ca+Cb)/(m*v1)));
H = (-v1-(2*(a*Ca-b*Cb)/(m*v1)) + L*2*((a*a*Ca+b*b*Cb)/(Iz*v1)))*rho*v1;
d = ((2*Ca)/m) + ((2*a*Ca)/Iz);
s1 = val_des1-x(i,2)+c1*(val_des-x(i,1));

u(i) = -1/d*(z1+g*(x(i,1))+H+c1*z1d+c2*s1-val_des2)-k*sign(s1); 
%u(i) = 1/33*(e*norm(x(i,:))^a*sign(s)+k*abs(s)^(b*sign(abs(s)-1))*s+c*(thd-x(2))+thdd+25*x(1));
end
figure(1)
plot(t,sin(t),'linewidth',1.5)
hold on
plot(t,x(:,1),':r','linewidth',1.5)
ylabel('\theta (rad)');xlabel('time(s)');
legend('ref signal','\theta value')
ylim([-6 6])
%title('Tracking con velocidad 100 m/s')
figure(2)
plot(t,u)
ylabel('u');xlabel('time(s)');
title('Ley de control')

%fileID = fopen('2.txt','w'); 
%y = [t' ;u' ]
%fprintf(fileID,'%10.3f %10.3f \n',y);
%save('Datos3.mat','x_1','y_1','z_1','x_2','y_2','z_2');
%fclose(fileID);

