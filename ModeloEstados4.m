%% ==========================================================
% PROGRAMA 04
% DISEÑO DE POLOS DESEADOS
%
% A partir de:
%     Mp
%     Ts
%
% Calcula:
%     zeta
%     wn
%     polos deseados
%     polinomio caracteristico
%% ==========================================================

clc
clear
close all

fprintf('\n');
fprintf('============================================\n');
fprintf(' DISENO DE POLOS DESEADOS\n');
fprintf('============================================\n\n');

%% INGRESO DE DATOS

Mp = input('Sobreimpulso Mp (%) = ');

Ts = input('Tiempo de establecimiento Ts (seg) = ');

fprintf('\n');

%% CALCULO DEL AMORTIGUAMIENTO

zeta = -log(Mp/100)/sqrt(pi^2 + (log(Mp/100))^2);

fprintf('Coeficiente de amortiguamiento\n');

fprintf('zeta = %.4f\n\n',zeta);

%% FRECUENCIA NATURAL

wn = 4/(zeta*Ts);

fprintf('Frecuencia natural\n');

fprintf('wn = %.4f rad/s\n\n',wn);

%% POLOS DESEADOS

sigma = -zeta*wn;

wd = wn*sqrt(1-zeta^2);

p1 = sigma + 1i*wd;

p2 = sigma - 1i*wd;

fprintf('Polos deseados\n\n');

disp(p1)

disp(p2)

%% POLINOMIO CARACTERISTICO

coef = poly([p1 p2]);

fprintf('\n');

disp('Polinomio caracteristico')

fprintf('\n');

fprintf('s^2 + %.4f s + %.4f\n',coef(2),coef(3));

%% GRAFICO

figure

plot(real([p1 p2]),imag([p1 p2]),...
    'rx',...
    'MarkerSize',12,...
    'LineWidth',2)

grid on

xlabel('Parte Real')

ylabel('Parte Imaginaria')

title('Polos Deseados')

xline(0,'k--')

yline(0,'k--')

axis equal

%% RESUMEN

fprintf('\n');

fprintf('============================================\n');

fprintf('RESUMEN DEL DISENO\n');

fprintf('============================================\n\n');

fprintf('Mp    = %.2f %%\n',Mp);

fprintf('Ts    = %.3f s\n',Ts);

fprintf('zeta  = %.4f\n',zeta);

fprintf('wn    = %.4f rad/s\n',wn);

fprintf('Polo1 = %.4f %+8.4fi\n',real(p1),imag(p1));

fprintf('Polo2 = %.4f %+8.4fi\n',real(p2),imag(p2));

fprintf('\n');

fprintf('Polinomio deseado:\n');

fprintf('s^2 + %.4f s + %.4f\n',coef(2),coef(3));

fprintf('\n');

fprintf('El siguiente programa calculara la matriz K.\n');