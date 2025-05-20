clc; close all; clear vars;

Ts = 0.02;
s = tf('s');
%Controlador proporcional
Cp = 0.2;

%Bode
%figure;


%Paso a digital
Cdp = c2d(tf(Cp), Ts, 'tustin');


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
%figure;
%Paso a digital
Cdpd = c2d(tf(Cpd), Ts, 'tustin');
