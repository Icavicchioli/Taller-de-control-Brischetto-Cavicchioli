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


%% Armo las matrices con accion integral
%En este caso, por el obs, tenemos todos los estados, por ende C = I

C = [1 0 0 0 ];
Cd = C;

% Armo la matriz combinacion de Ad y Cd (nota: Consultar col de unos)

Ad_aug = [Ad , zeros([4,1]);
       -Cd*Ts , ones([1,1])];
   
Bd_aug = [Bd; 0];

%Hago el pole placement para hallar K_aug (el ultimo es k_integral)
polos_nuevos = [-4+2i ; -4-2i ; -12.2957+1.80971i; -12.2957-1.80971i; -2];
polos_discretos = exp(Ts*polos_nuevos);

K_aug = place(Ad_aug, -Bd_aug, polos_discretos);


   

