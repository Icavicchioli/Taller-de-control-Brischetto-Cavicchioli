% Datos
t = out.tout;
posicion = out.dist_obs;
referencia = out.ref(:,1);
velocidad = out.vel_obs;
angulo = out.angulo_obs;
velocidad_angular = out.velang_obs;

% Crear figura
figure('Units','normalized','Position',[0.1 0.1 0.6 1.2])  % antes 0.8, ahora 1.2
% Posición
subplot(4,1,1)
plot(t, posicion, 'b', 'LineWidth', 1.2)
hold on
plot(t, referencia, 'r--', 'LineWidth', 1.2)
ylabel('Posición [m]')
legend('Observada','Referencia')
grid on
title('Respuesta del sistema')

% Velocidad
subplot(4,1,2)
plot(t, velocidad, 'b', 'LineWidth', 1.2)
ylabel('Velocidad [m/s]')
grid on

% Ángulo
subplot(4,1,3)
plot(t, angulo, 'b', 'LineWidth', 1.2)
ylabel('Ángulo [rad]')
grid on

% Velocidad angular
subplot(4,1,4)
plot(t, velocidad_angular, 'b', 'LineWidth', 1.2)
xlabel('Tiempo [s]')
ylabel('Vel. ang. [rad/s]')
grid on

% Guardar en PDF (versión MATLAB 2018)
print(gcf, 'graficos_observador_simulink', '-dpdf', '-bestfit')
