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
polos_continuos = -[4 4 3.5 3.5] *12.4289;  %[-36 -36 -36 -36]
polos_discretos = exp(polos_continuos*Ts);

figure();
pzmap(tf(zpk([],polos_continuos,1)), "r", tf(P))
grid on;
legend("Observador", "Planta")


%%
% chequeamos controlabilidad y observabilidad
rank(ctrb(Ad,Bd))
rank(obsv(Ad,Cd))


%%
% ahora que tenemos los polos buscamos el L discreto
 
Ld = place(Ad',Cd',polos_discretos)'
eig(Ad - Ld*Cd);

%% Graficos hechos con obs -[4 4 3.5 3.5] *12.4289
clc ;close all;
muestras = length(out.posicion);
t = 0:Ts:(muestras-1)*Ts;

figure();
plot(t, out.posicion)
xlabel("tiempo[s]")
ylabel("distancia[m]")
legend("Medicion", "Estimacion")
xlim([0, 5])
grid on;
saveas(gcf,'img_obs/posicion_obs.png')

%velocidad
t1 = 0:Ts:(length(t)-1-1)*Ts;

velocidad_estimada = out.velocidad(:, 2);
velocidad_medida = diff(out.posicion(:, 1))./diff(t1);
velocidad_medida = velocidad_medida(:, 1);

figure();
plot(t1, velocidad_medida)
hold on;
plot(t, velocidad_estimada)
xlabel("tiempo[s]")
ylabel("velocidad[m/s]")
legend("Medicion", "Estimacion")
xlim([0, 5])
grid on;
saveas(gcf,'img_obs/velocidad_obs.png')

figure();
plot(t, out.angulo)
xlabel("tiempo[s]")
ylabel("angulo[rad]")
legend("Medicion", "Estimacion")
xlim([0, 5])
grid on;
saveas(gcf,'img_obs/angulo_obs.png')

figure();
plot(t, out.velocidad_angular)
xlabel("tiempo[s]")
ylabel("velocidad angular[rad/s]")
legend("Medicion", "Estimacion")
xlim([0, 5])
grid on;
saveas(gcf,'img_obs/velocidad_angular_obs.png')

figure();
plot(t, out.u)
xlabel("tiempo[s]")
ylabel("angulo[rad]")
xlim([0, 5])
grid on;
saveas(gcf,'img_obs/u_obs.png')



