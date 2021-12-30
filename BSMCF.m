%Este código fue originalmente modificado de:
%https://www.youtube.com/watch?v=abwaZgiST3c&t=695s 
%Creditos totales al autor.
function dx = BSMCF(t,x)% t,x entradas y dx=x1dot,x2dot salidas

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

val_des = sin(t); %Valor deseado.
val_des1 = cos(t); %Primera derivada del valor deseado.
val_des2 = -sin(t); %Segunda derivada del valor deseado.

z1 = -val_des+x(1); %Definimos el error.
z1d = -val_des1+x(2); %Derivada del error.
s = val_des1-x(2)+c1*z1; %Superficie deslizante.
f = (-2*L*((a*Ca + b*Cb)/(Iz*v1)) - (2*(Ca+Cb)/(m*v1))); %Alpha 32.
H = (-v1-(2*(a*Ca-b*Cb)/(m*v1)) + L*2*((a*a*Ca+b*b*Cb)/(Iz*v1)))*rho*v1; %Disturbio, perturbación.
d = ((2*Ca)/m) + ((2*a*Ca)/Iz); %Alpha 35.
u = -1/d*(z1+f*(x(1))+H+c1*z1d+c2*s-val_des2)-k*sign(s); %Ley de control.

x1dot = x(2); %Espacio de estados.
x2dot = f*x(1)+d*u+H; %Espacio de estados.
dx = [x1dot;x2dot]; %Salidas.
end


