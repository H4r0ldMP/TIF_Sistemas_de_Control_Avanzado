%% 29/12/2021 
%% Programa Original del libro Sliding Mode Control Using MATLAB 
%% Modificado por Kevin L.
function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[-2 -2];
str=[];
ts=[];
function sys=mdlDerivatives(t,x,u)
Ca =67;
Cb =67;
a =1.1;
b =1.67;
Iz =2035;
L =4;
m =1525;
v =40;
p =0.01;

l = 2*Ca/m+2*a*Ca/Iz;%% a35
ll = (-2*L*((a*Ca + b*Cb)/(Iz*v)) - (2*(Ca+Cb)/(m*v)));%% a32
lll = ((2*L*(a*Ca + b*Cb))-(2*(a*Ca-b*Cb)))/(m*v); % a34
q = (-v-2*(a*Ca-b*Cb)/(m*v)+L*(2*(a*a*Ca+b^2*Cb)/(Iz*v)))*p*v; %B[3]

sys(1)=x(2); %% x_dot_1
sys(2)=ll*x(2)+l*u+q; %% x_dot_2) 
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);