% Definís el rango de tiempo que querés mostrar (en segundos)
t_min = 65;  % ej. desde 10 s
t_max = 155;  % hasta 20 s

% Filtrar índices dentro del rango
idx = (out.tout >= t_min) & (out.tout <= t_max);

% Extraer los datos filtrados
t = out.tout(idx);
posicion = out.posicion(idx,2);
referencia = out.posicion(idx,3);
velocidad = out.velocidad(idx,2);
angulo = out.angulo(idx,2);
velocidad_angular = out.velocidad_angular(idx,2);

% Crear figura más alta
figure('Units','normalized','Position',[0.1 0.1 0.6 1.2])

% Posición
subplot(4,1,1)
plot(t, posicion, 'b', 'LineWidth', 1.2)
hold on
plot(t, referencia, 'r--', 'LineWidth', 1.2)
ylabel('Posición [m]')
legend('Observada','Referencia')
grid on
title(sprintf('Respuesta del sistema entre %.2f y %.2f segundos', t_min, t_max))

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

% Ajustar tamaño para PDF
set(gcf, 'PaperUnits', 'inches')
set(gcf, 'PaperPosition', [0 0 8 10])

% Guardar PDF
print(gcf, 'graficos_observador_filtrado', '-dpdf')
