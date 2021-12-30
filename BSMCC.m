%Este código fue originalmente modificado de:
%https://www.youtube.com/watch?v=abwaZgiST3c&t=695s 
%Creditos totales al autor.
clc; clear all; close all;
[t,x] = ode45(@BSMCF,[0 20],[-2 -2]); %Llamamos a la función BSMCF con ode45.
u = zeros(length(t),1); 
c1 = 15; %Parámetros de la ley de aproximación modos deslizantes.
c2 = 12; %Parámetros de la ley de aproximación modos deslizantes.
k = 10; %Factor multiplicativo de la función signo.
L = 4; %Longitud del carro(m).
v1 = 20; %velocidad(km/h).
rho = 0.1; %Curvatura de la carretera en el punto de vista previa.
m = 1525; %Masa del vehiculo(kg).
a = 1.1;  %Distancia del centro del carro a la rueda delantera(m).
b = 1.67; %Distancia del centro del carro a la rueda trasera(m).
Ca = 67; %Rigidez en curvas de la rueda delantera(kN/rad).
Cb = 67; %Rigidez en curvas de la rueda trasera(kN/rad).
Iz = 2305; %Inercia de guiñada(kg/m^2).
for i=1:length(t) %Bucle for para integrar la ecuación de la desviaión lateral del vehículo.

val_des = sin(t(i)); %Valor deseado.
val_des1 = cos(t(i)); %Primera derivada del valor deseado.
val_des2 = -sin(t(i)); %Segunda derivada del valor deseado.

z1 = x(i,1)- val_des; %Definimos el error.
z1d = x(i,2)- val_des1; %Derivada del error.
f = (-2*L*((a*Ca + b*Cb)/(Iz*v1)) - (2*(Ca+Cb)/(m*v1))); %Alpha 32.
H = (-v1-(2*(a*Ca-b*Cb)/(m*v1)) + L*2*((a*a*Ca+b*b*Cb)/(Iz*v1)))*rho*v1; %Disturbio, perturbación.
d = ((2*Ca)/m) + ((2*a*Ca)/Iz); %Alpha 35.
s = val_des1-x(i,2)+c1*(val_des-x(i,1)); %Superficie deslizante.

u(i) = -1/d*(z1+f*(x(i,1))+H+c1*z1d+c2*s-val_des2)-k*sign(s); %Ley de control.

end
figure(1)
plot(t,sin(t),'linewidth',1.5) %Graficamos un seno(t) respecto al tiempo.
hold on %Sobreponemos ambas gráficas.
plot(t,x(:,1),':r','linewidth',1.5) %Graficamos una columna de x respecto al tiempo.
ylabel('\theta (rad)');xlabel('time(s)'); %Eje y(theta), eje x(tiempo).
legend('ref signal','\theta value') %Leyenda.
ylim([-6 6]) %Limites en el eje y.

figure(2)
plot(t,u) %Graficamos la ley de control respecto al tiempo.
ylabel('u');xlabel('time(s)'); %Eje y(ley de control u), eje x(tiempo).

fileID = fopen('100.txt','w'); 
y = [t' ;x(:,1)'; u']
fprintf(fileID,'%f %f %f \n',y);
fclose(fileID);

