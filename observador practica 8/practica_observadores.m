clc
clear all
% quierlo que los polos del observador estén en 3 veces approx los de la
% planta, así que los llevo a 36 rad

% el espacio de estados identificado de la planta con los polos en -12.43 es el siguiente:
% X = [tita ; tita´]

A = [0 1 ;-154.5 -24.86];
B = [0 ; 64.89];
C = [1 0];
D = 0;

Cd = C;

P = ss(A,B,C,D);

L = acker(A',C',[-36 -36])';

M = A-L*C ; %matriz del observador - los autovalores coinciden con los polos elegidos.
eig(M);     %verificamos autovalores
% entonces:

Ts = 0.02;

Ad = eye(2) + Ts*A; 
Bd = P.B*Ts;

Ld = exp(Ts*L);

Ld = acker(Ad',Cd',[0.486 0.486])';

