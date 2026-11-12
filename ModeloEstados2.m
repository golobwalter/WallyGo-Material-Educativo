%% ==========================================================
% PROGRAMA 02
% ANALISIS DE CONTROLABILIDAD
%% ==========================================================

clc
clear
close all

fprintf('\n');
fprintf('=========================================\n');
fprintf(' ANALISIS DE CONTROLABILIDAD\n');
fprintf('=========================================\n');

%% MATRICES

A = [0 1;
     0 -11.25];

B = [0;
     125];

%% Orden del sistema

n = size(A,1);

fprintf('\n');
fprintf('Orden del sistema = %d\n',n);

%% Construccion de la matriz de controlabilidad

Co = [];

for k=0:n-1

    Co = [Co A^k*B];

end

fprintf('\n');
disp('Matriz de Controlabilidad:')

disp(Co)

%% Rango

r = rank(Co);

fprintf('\n');
fprintf('Rango = %d\n',r);

%% Verificacion

if r==n

    fprintf('\n');
    fprintf('****************************************\n');
    fprintf(' EL SISTEMA ES COMPLETAMENTE CONTROLABLE\n');
    fprintf('****************************************\n');

else

    fprintf('\n');
    fprintf('****************************************\n');
    fprintf(' EL SISTEMA NO ES CONTROLABLE\n');
    fprintf('****************************************\n');

end

%% Explicacion

fprintf('\n');
fprintf('Interpretacion:\n');
fprintf('\n');

fprintf('Cada columna representa como la entrada\n');
fprintf('va afectando los estados del sistema.\n');

fprintf('\n');

fprintf('Si el rango coincide con el numero\n');
fprintf('de estados, es posible mover el sistema\n');
fprintf('a cualquier estado mediante la entrada.\n');

fprintf('\n');

fprintf('Numero de estados = %d\n',n);
fprintf('Rango obtenido    = %d\n',r);

fprintf('\n');

if r==n

    fprintf('Se puede continuar con el diseno del controlador.\n');

else

    fprintf('No es posible realizar asignacion de polos.\n');

end