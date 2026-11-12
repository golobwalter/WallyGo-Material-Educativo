
clc;
clear;
close all;

%% 1. MODELO DEL MOTOR CC

G = tf(125,[1 11.25 0]);

fprintf('\nFUNCION DE TRANSFERENCIA DEL MOTOR:\n');
G

%% 2. ESPECIFICACIONES DE DISEÑO

Mp = 5;       % Sobreimpulso porcentual
Ts = 3;       % Tiempo de establecimiento (2%%)
p3 = -10;     % Tercer polo elegido

%% 3. CALCULO DE ZETA Y Wn

zeta = -log(Mp/100) / ...
       sqrt(pi^2 + log(Mp/100)^2);

wn = 4/(zeta*Ts);

%% 4. POLINOMIO DE SEGUNDO ORDEN

a1 = 2*zeta*wn;
a0 = wn^2;

fprintf('\nPOLINOMIO DE SEGUNDO ORDEN:\n');
fprintf('s^2 + %.5f s + %.5f\n',a1,a0);

%% 5. AGREGAR EL TERCER POLO

P2 = [1 a1 a0];
P3 = conv(P2,[1 -p3]);

fprintf('\nPOLINOMIO DE TERCER ORDEN:\n');
disp(P3);

%% 6. CALCULO DE LAS GANANCIAS PID

% Polinomio real:
% s^3 + (11.25+125*Kd)s^2
%     +125*Kp*s +125*Ki

Kd = (P3(2)-11.25)/125;
Kp = P3(3)/125;
Ki = P3(4)/125;

fprintf('\nGANANCIAS DEL PID:\n');
fprintf('Kp = %.6f\n',Kp);
fprintf('Ki = %.6f\n',Ki);
fprintf('Kd = %.6f\n',Kd);

%% 7. CONTROLADOR PID

C = pid(Kp,Ki,Kd);

%% 8. SISTEMA EN LAZO CERRADO

T = feedback(C*G,1);

fprintf('\nFUNCION DE TRANSFERENCIA EN LAZO CERRADO:\n');
T

%% 9. POLOS DEL SISTEMA

fprintf('\nPOLOS DEL LAZO CERRADO:\n');
disp(pole(T));

%% 10. RESPUESTA TEMPORAL

figure;
step(T,8);
grid on;
title('Respuesta al escalon - Control PID');
xlabel('Tiempo (s)');
ylabel('Posicion angular');

%% 11. CARACTERISTICAS DE LA RESPUESTA

info = stepinfo(T);

fprintf('\nCARACTERISTICAS DE LA RESPUESTA:\n');
fprintf('Sobreimpulso = %.2f %%\n',info.Overshoot);
fprintf('Tiempo establecimiento = %.3f s\n', ...
        info.SettlingTime);
fprintf('Tiempo de subida = %.3f s\n',info.RiseTime);

%% 12. COMPARACION CON EL MODELO DE SEGUNDO ORDEN

T2 = tf(wn^2,[1 2*zeta*wn wn^2]);

figure;
step(T,T2,8);
grid on;
legend('Motor con PID','Modelo de segundo orden');
title('Comparacion de respuestas');
xlabel('Tiempo (s)');
ylabel('Posicion angular');
