clc; close all; clear vars;

Ts = 0.02;
s = tf('s');

P = (569.88) * (1/s) * (1/(s+3.81)) * (1/(s^2 + 20.74*s + 154.5));

%Controlador proporcional
Cp = 3.2;

%Bode
figure;
bode(Cp*P);
grid();
title("Proporcional")


%Paso a digital
Cdp = c2d(tf(Cp), Ts, 'tustin');


%% Control proporcional integral
Cpi = pid(2.5, 0.5, 0);

%Bode
figure;
bode(Cpi*P);
grid();
title("Proporcional Integral")

%Paso a digital
Cdpi = c2d(tf(Cpi), Ts, 'tustin');

%% Control proporcional derivativo
Cpd = pid(2.6, 0, 0.0012);

%Bode
figure;
bode(Cpd*P);
grid();
title("Proporcional Derivativo")

%Paso a digital
Cdpd = c2d(tf(Cpd), Ts, 'tustin');


%% Graficos
muestras = 400;
t = 0:Ts:(muestras-1)*Ts;

%Proporcional
figure();
plot(t, -step_prop(1:muestras))
hold
plot(t, step_prop_sim(1:muestras))
title("Escalon 0.1 proporcional");
grid;
legend("Planta real", "Simulink")
xlabel("Tiempo[s]")
ylabel("Posicion [m]")

muestras = 400;
t = 0:Ts:(muestras-1)*Ts;

figure();
plot(t, impulso_prop(1:muestras))
hold
plot(t, impulso_prop_sim(1:muestras))
title("Impulso 0.05 proporcional");
legend("Planta real", "Simulink")
grid;
xlabel("Tiempo[s]")
ylabel("Posicion [m]")

%Proporcional integral
muestras = 1000;
t = 0:Ts:(muestras-1)*Ts;

figure();
plot(t, -step_pi_v2(1:muestras))
hold
plot(t, step_pi_sim(1:muestras))
title("Escalon 0.1 proporcional integral");
legend("Planta real", "Simulink")
grid;
xlabel("Tiempo[s]")
ylabel("Posicion [m]")

%Proporcional Derivativo
muestras = 700;
t = 0:Ts:(muestras-1)*Ts;

figure();
plot(t, -step_pd(1:muestras))
hold
plot(t, step_pd_sim(1:muestras))
title("Escalon 0.1 proporcional derivativo");
grid;
legend("Planta real", "Simulink")
xlabel("Tiempo[s]")
ylabel("Posicion [m]")
muestras = 500;
t = 0:Ts:(muestras-1)*Ts;

figure();
plot(t, impulso_pd(1:muestras))
hold
plot(t, impulso_pd_sim(1:muestras))
title("Impulso 0.05 proporcional derivativo");
legend("Planta real", "Simulink")
grid;
xlabel("Tiempo[s]")
ylabel("Posicion [m]")
