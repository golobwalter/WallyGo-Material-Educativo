%% ==========================================================
% PROGRAMA 07
% DISEÑO DEL OBSERVADOR DE LUENBERGER
%
% Calcula la matriz L mediante asignación de polos
% utilizando la función PLACE de MATLAB.
%% ==========================================================

clc
clear
close all

fprintf('\n');
fprintf('============================================\n');
fprintf(' DISENO DEL OBSERVADOR DE LUENBERGER\n');
fprintf('============================================\n\n');

%% MATRICES DEL SISTEMA

A = [0 1;
     0 -11.25];

B = [0;
     125];

C = [1 0];

%% POLOS DEL CONTROLADOR

Pc = [-1.3333+1.3986i
      -1.3333-1.3986i];

fprintf('Polos del controlador\n\n');

disp(Pc)

%% FACTOR DE RAPIDEZ DEL OBSERVADOR

factor = input('Factor de rapidez del observador = ');

fprintf('\n');

%% POLOS DEL OBSERVADOR

Po = factor * Pc;

fprintf('Polos del observador\n\n');

disp(Po)

%% CALCULO DE LA MATRIZ L

L = place(A',C',Po)';

fprintf('\n');

fprintf('Matriz L\n\n');

disp(L)

%% VERIFICACION

Aobs = A - L*C;

lambda = eig(Aobs);

fprintf('\n');

fprintf('Polos del observador\n\n');

disp(lambda)

%% POLINOMIO CARACTERISTICO

coef = poly(lambda);

fprintf('\n');

fprintf('Polinomio caracteristico\n\n');

fprintf('s^2 + %.4f s + %.4f\n',coef(2),coef(3));

%% GRAFICO

figure

plot(real(lambda),imag(lambda),...
    'rx',...
    'MarkerSize',12,...
    'LineWidth',2)

grid on

xlabel('Parte Real')

ylabel('Parte Imaginaria')

title('Polos del Observador')

xline(0,'k--')

yline(0,'k--')

axis equal

%% RESUMEN

fprintf('\n');

fprintf('============================================\n');
fprintf('RESUMEN\n');
fprintf('============================================\n\n');

fprintf('Factor de rapidez = %.2f\n\n',factor);

fprintf('Polos del observador:\n\n');

fprintf('Polo 1 = %.4f %+8.4fi\n',...
    real(lambda(1)),imag(lambda(1)));

fprintf('Polo 2 = %.4f %+8.4fi\n',...
    real(lambda(2)),imag(lambda(2)));

fprintf('\n');

fprintf('Matriz L:\n\n');

disp(L)

fprintf('El siguiente programa verificara la matriz L.\n');