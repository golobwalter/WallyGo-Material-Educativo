%% ============================================================
%  PROGRAMA 01
%  ANALISIS DEL MODELO EN ESPACIO DE ESTADOS
%
%  Autor: Wally + ChatGPT
%
%  Objetivo:
%  Analizar un sistema representado por las matrices
%  A, B, C y D.
%% ============================================================

clc
clear
close all

fprintf('\n');
fprintf('=============================================\n');
fprintf(' ANALISIS DEL MODELO EN ESPACIO DE ESTADOS\n');
fprintf('=============================================\n\n');

%% MATRICES DEL SISTEMA

A = [0 1;
     0 -11.25];

B = [0;
     125];

C = [1 0];

D = 0;

%% Mostrar matrices

disp('Matriz A =')
disp(A)

disp('Matriz B =')
disp(B)

disp('Matriz C =')
disp(C)

disp('Matriz D =')
disp(D)

%% Dimensiones

n = size(A,1);
m = size(B,2);
p = size(C,1);

fprintf('\n');
fprintf('Numero de estados  : %d\n',n);
fprintf('Numero de entradas : %d\n',m);
fprintf('Numero de salidas  : %d\n',p);

%% Verificaciones

fprintf('\n');
fprintf('Verificando dimensiones...\n');

ok = true;

if size(A,1)~=size(A,2)

    disp('ERROR: A no es cuadrada')
    ok=false;

end

if size(B,1)~=size(A,1)

    disp('ERROR: filas de B incorrectas')
    ok=false;

end

if size(C,2)~=size(A,2)

    disp('ERROR: columnas de C incorrectas')
    ok=false;

end

if size(D,1)~=size(C,1)

    disp('ERROR: filas de D incorrectas')
    ok=false;

end

if size(D,2)~=size(B,2)

    disp('ERROR: columnas de D incorrectas')
    ok=false;

end

if ok

    disp('Modelo correcto.')

end

%% Crear sistema

sys = ss(A,B,C,D);

fprintf('\n');
disp('Sistema en espacio de estados:')

sys

%% Funcion de transferencia

fprintf('\n');
disp('Funcion de transferencia equivalente:')

tf(sys)

%% Explicacion

fprintf('\n');
fprintf('---------------------------------------------\n');
fprintf('INTERPRETACION FISICA\n');
fprintf('---------------------------------------------\n');

fprintf('\n');

fprintf('A : dinamica natural del sistema\n');

fprintf('B : como actua la entrada sobre el sistema\n');

fprintf('C : estados que se observan como salida\n');

fprintf('D : paso directo entrada-salida\n');

fprintf('\n');
fprintf('Ecuacion de estado:\n');

fprintf('      x_dot = A*x + B*u\n');

fprintf('\n');

fprintf('Ecuacion de salida:\n');

fprintf('      y = C*x + D*u\n');

fprintf('\n');
fprintf('Fin del Programa 01\n');