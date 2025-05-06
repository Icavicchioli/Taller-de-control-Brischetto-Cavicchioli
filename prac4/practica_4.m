clear all;
close all;
s=tf('s');
z=tf('z',0.02);


P=zpk([],[-7.7416 + 5.6394i , -7.7416 - 5.6394i],0.42*91.74);

%P=zpk([],- abs([-7.7416 + 5.6394i , -7.7416 - 5.6394i]),0.42*91.74);


C = db2mag(4)*(s+9.5779)/s;

Ts=0.02;
P_retardo=  (1 - Ts/4 * s)/(1 + Ts/4 * s); %La transferencia

PP = P_retardo*P;

L = P*C*P_retardo;

%%

CdTustin = c2d(C,Ts,'tustin')

CdFOH = c2d(C,Ts,'foh')



S=1/(1+L);
T=1-S;
PS=minreal(P*S);
CS=minreal(C*S);

