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

Ts = 0.02;

sys_d = c2d(ss(A,B,C,D), Ts);

Cd = C;
Ad = sys_d.A;
Bd = sys_d.B;

%%

% bucamos que los polos del observador sean rápidos (3-5 veces)
polos_continuos = -[3 3.05 3.1 3.025] *12.4289  % ~[-36 -36 -36 -36]
polos_discretos = exp(polos_continuos*Ts)

%%
% chequeamos controlabilidad y observabilidad
rank(ctrb(Ad,Bd))
rank(obsv(Ad,Cd))


%%
 % ahora que tenemos los polos buscamos el L discreto
 
Ld = place(Ad',Cd',polos_discretos)'

%% Prueba que funcione el observador - gracias chatgpt

N = 100;
x_real = zeros(4,N);
x_est  = zeros(4,N);
y = zeros(2,N);
u = ones(1,N); % o algún pulso

for k = 1:N-1
    y(:,k) = Cd * x_real(:,k);
    x_est(:,k+1) = Ad * x_est(:,k) + Bd * u(k) + Ld * (y(:,k) - Cd * x_est(:,k));
    x_real(:,k+1) = Ad * x_real(:,k) + Bd * u(k);
end

plot(x_real'); hold on; plot(x_est','--');
legend('x1','x2','x3','x4','x1 est','x2 est','x3 est','x4 est');

%%
s=tf('s');
filtro = (s+20*pi*2);
filtrod=c2d(filtro,0.02,'tustin')
