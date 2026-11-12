%% ==========================================================
% PROGRAMA 05
% CALCULO SIMBOLICO DE LA MATRIZ K
%
% Calcula K por comparación de polinomios
% SIN usar place() ni acker()
%
% Autor: Wally + ChatGPT
%% ==========================================================

clc
clear
close all

fprintf('\n');
fprintf('=====================================================\n');
fprintf(' CALCULO SIMBOLICO DE LA MATRIZ K\n');
fprintf('=====================================================\n\n');

%% MATRICES DEL SISTEMA

A = [0 1;
     0 -11.25];

B = [0;
     125];

%% POLINOMIO DESEADO

a1 = 2.6667;
a0 = 3.7329;

fprintf('Polinomio deseado\n');
fprintf('P(s)= s^2 + %.4f s + %.4f\n\n',a1,a0);

%% VARIABLES SIMBOLICAS

syms k1 k2 s

K = [k1 k2];

fprintf('Matriz K simbolica\n');
disp(K)

%% MATRIZ BK

BK = B*K;

fprintf('\n');
fprintf('Matriz BK\n');

disp(BK)

%% MATRIZ A-BK

Acl = A-BK;

fprintf('\n');
fprintf('Matriz A-BK\n');

disp(Acl)

%% MATRIZ sI-(A-BK)

SI = s*eye(size(A));

M = SI-Acl;

fprintf('\n');
fprintf('Matriz sI-(A-BK)\n');

disp(M)

%% DETERMINANTE

P = expand(det(M));

fprintf('\n');
fprintf('Polinomio caracteristico obtenido\n');

disp(P)

%% COEFICIENTES

coef = coeffs(P,s,'All');

fprintf('\n');
fprintf('Coeficientes encontrados\n');

disp(coef)

%% ECUACIONES

eq1 = coef(2)==a1;

eq2 = coef(3)==a0;

fprintf('\n');
fprintf('Sistema de ecuaciones\n');

disp(eq1)

disp(eq2)

%% RESOLVER

sol = solve([eq1 eq2],[k1 k2]);

k1 = double(sol.k1);

k2 = double(sol.k2);

%% RESULTADOS

fprintf('\n');
fprintf('=====================================\n');
fprintf(' MATRIZ K\n');
fprintf('=====================================\n\n');

fprintf('k1 = %.6f\n',k1);

fprintf('k2 = %.6f\n',k2);

fprintf('\n');

fprintf('K = [ %.6f   %.6f ]\n',k1,k2);

fprintf('\n');
fprintf('Fin del programa\n');