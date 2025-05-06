clc; close all; clear vars;

%Constantes
g = 9.81;
m = 0;
Ts = 20e-3;

s = tf('s');
% (s+25*180/pi) % polo en alta frec, 24hz
%Planta total
%P_servo = zpk([],[-7.7416 + 5.6394i , -7.7416 - 5.6394i],0.42*91.74); %caso sin carro
P_servo = zpk([],[ -10.3699 + 6.8515i , -10.3699 - 6.8515i], 54.92 +9.97 ); %caso con carro - el +9.97 hace que en reposo vaya a los 0.42 de la relación encontrada - se lo sa

% PI y sus coefs
Kp=1.24;
Ki=16.2;
PI_planta_servo = Kp + Ki/s; % controlador de servo a barra

P_servo_total = tf(PI_planta_servo) * P_servo; % la planta nueva que incluye el control del ángulo de la barra

P_carro = g/(s^2);
retardo = exp(-s*Ts/2);
P = P_servo*P_carro*retardo;

%%

%Controlador proporcional
%C = db2mag(-92);

%Bode
%figure;
%bode(C*P)

%Paso a digital
%Cd = c2d(tf(C), Ts, 'tustin');

%T = C*P/(1+(C*P));

%step(0.25*T)

%% Control proporcional integral

Cpi = db2mag(0)*(1/s)*(s+1);

%Bode
%figure;
%bode(Cpi*P)

%Paso a digital
Cdpi = c2d(tf(Cpi), Ts, 'tustin');

%% Control proporcional derivativo

Cpd = db2mag(-5.4)*(s+1)/(s+10); %agregue un polo para regularizar

%Bode
figure;
bode(Cpd*P)

%Paso a digital
Cdpd = c2d(tf(Cpd), Ts, 'tustin');



