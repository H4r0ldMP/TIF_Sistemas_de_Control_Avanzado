%% 29/12/2021 
%% Programa Original del libro Sliding Mode Control Using MATLAB 
%% Modificado por Kevin L.
function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];
function sys=mdlOutputs(t,x,u)
%%----------------  CONSTANTES
ka =15;%%k11
kb =12;%%k12

Ca =67;
Cb =67;
a =1.1;
b =1.67;
Iz =2035;
L =4;
m =1525;
v =40;
p =0.01;
M =10 ;

l = 2*Ca/m+2*a*Ca/Iz;%% a35
ll = (-2*L*((a*Ca + b*Cb)/(Iz*v)) - (2*(Ca+Cb)/(m*v)));%% a32
q = (-v-2*(a*Ca-b*Cb)/(m*v)+L*(2*(a^2*Ca+b^2*Cb)/(Iz*v)))*p*v; %B[3]
%%---------------------------------

x1=u(2); %% Estado actual - X(1)
xd=u(1); %% Estado deseado - SEÑAL SENOIDAL
dx1 = u(3); % Derivada del estado actual ( X(2) )
dxd = cos(t); % Primera derivada del estado deseado
ddxd = -sin(t); % Segunda derivada del estado deseado

e = x1 - xd; % error = estado actual - estado deseado
de = dx1 - dxd; % derivada del error

s = dx1 + ka*e - xd; % Superficie deslizante = e2

fx = ll*dx1; % f(x)

ut = -(1/l)*(fx+q+ka*de-ddxd+e+kb*s)-M*sign(s); % LEY DE CONTROL

sys(1)=ut;
sys(2)=e;
sys(3)=de;
