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

% bucamos que los polos del observador sean rápidos (3-5 veces)
polos_continuos = -[4 4 3.5 3.5] *12.4289  %[-36 -36 -36 -36]
polos_discretos = exp(polos_continuos*Ts)

%%
% chequeamos controlabilidad y observabilidad
rank(ctrb(Ad,Bd))
rank(obsv(Ad,Cd))


%%
% ahora que tenemos los polos buscamos el L discreto
 
Ld = place(Ad',Cd',polos_discretos)'
eig(Ad - Ld*Cd);
