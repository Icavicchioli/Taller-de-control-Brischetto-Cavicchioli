clear all
clc

A = [
[0 1 0 0];
[0 -19.0476*0.2 8.78229 0];
[0 0 0 1];
[0 0 -154.478 -20.74];
];
B = [0 ;0;0; 64.89];
C =[1 0 0 0;
    0 0 1 0]; % sale con pos y angulo
D = 0;

P = ss(A,B,C,D);
Ts = 0.02;
Cd = C;
Ad = eye(4) + Ts*A; 
Bd = B*Ts;

%%
% chequeamos controlabilidad y observabilidad
rank(ctrb(Ad,Bd));
rank(obsv(Ad,Cd));

%% Polos del controlador por state feedback (nota: mas lentos que el obs)
polos_nuevos = [-4+2i ; -4-2i ; -12.2957+1.80971i; -12.2957-1.80971i];
polos_discretos = exp(Ts*polos_nuevos);

%% Calculo la ganancia de realimentacion
K = place(Ad, -Bd, polos_discretos);

%% Termino de feedthrough (nota: Consultar la segunda inversion)
F = 1/(Cd * inv(eye(4) - (Ad + Bd*K)) * Bd);


