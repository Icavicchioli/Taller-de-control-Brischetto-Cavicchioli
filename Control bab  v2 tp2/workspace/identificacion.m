clc; clear vars; close all;

p = respuesta(1:45)* 0.001;

Fc = 20;
Fs = 50;

%[b, a] = butter(1, Fc/(Fs/2), 'low');
%p = filter(b, a, p);

v = diff(p);
a = diff(v);

figure();
plot(p);
title("posicion");
figure();
plot(v);
title("velocidad");
figure();
plot(a);
title("Aceleracion");

tres = transpose(3*ones(43,1))
vels = transpose(-v/0.0525);
vels = vels(1:43)

X = transpose([tres ; vels]);
Y = a;

alfa = inv(transpose(X) * X) * transpose(X) * Y