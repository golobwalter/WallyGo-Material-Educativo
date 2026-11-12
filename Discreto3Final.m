clc;
clear;
close all;

%% =========================================================
%       COMPARACION DE POLOS - CONTROL DE POSICION
%
%   Este programa permite probar diferentes polos deseados
%   sin modificar el modelo de la planta.
%
%   ENTRADAS:
%       A, B, C, D
%       Ts
%       Diferentes polos deseados
%
%   CALCULA:
%       Ad, Bd
%       Polos discretos
%       K1, K2
%       Kr
%       L1, L2
%       Tiempo de subida
%       Tiempo de establecimiento
%       Sobreimpulso
%
% =========================================================


%% =========================================================
% 1. MODELO DE LA PLANTA
% =========================================================

A = [0      1;
     0   -11.5];

B = [0;
     125];

C = [1 0];

D = 0;

Ts = 0.01;       % 10 ms


%% =========================================================
% 2. DISCRETIZACION
% =========================================================

sys_c = ss(A,B,C,D);

sys_d = c2d(sys_c,Ts,'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;


%% =========================================================
% 3. DEFINIR LOS CASOS A COMPARAR
% =========================================================
%
% Cada fila contiene:
%
% [parte_real   parte_imaginaria]
%
% El programa genera automaticamente
% los polos conjugados.
%
% =========================================================


casos = [

   -1.0   1.0
   -1.3333  1.3983
   -1.5   1.5
   -2.0   2.0
   -2.5   2.5
   -3.0   3.0
];


numeroCasos = size(casos,1);


%% =========================================================
% 4. VECTOR DE TIEMPO PARA LA SIMULACION
% =========================================================

tiempo = 0:Ts:10;

referencia = 1;


%% =========================================================
% 5. MATRICES PARA GUARDAR RESULTADOS
% =========================================================

resultados = zeros(numeroCasos,10);

%
% Columnas:
%
% 1  Caso
% 2  Parte real
% 3  Parte imaginaria
% 4  K1
% 5  K2
% 6  Kr
% 7  Tiempo subida
% 8  Tiempo establecimiento
% 9  Sobreimpulso
% 10 Error estacionario
%


%% =========================================================
% 6. FIGURA
% =========================================================

figure;

hold on;

grid on;


%% =========================================================
% 7. CALCULO DE CADA CASO
% =========================================================

for i = 1:numeroCasos

    % -----------------------------------------------------
    % Polos continuos
    % -----------------------------------------------------

    parteReal = casos(i,1);

    parteImag = casos(i,2);


    p_cont = [
        parteReal + parteImag*1i;
        parteReal - parteImag*1i
    ];


    % -----------------------------------------------------
    % Polos discretos
    % -----------------------------------------------------

    p_disc = exp(p_cont*Ts);


    % -----------------------------------------------------
    % Controlador
    % -----------------------------------------------------

    Kd = place(Ad,Bd,p_disc);

    K1 = Kd(1);

    K2 = Kd(2);


    % -----------------------------------------------------
    % Ganancia de referencia
    % -----------------------------------------------------

    Kr = 1/(Cd*((eye(size(Ad))-Ad+Bd*Kd)\Bd));


    % -----------------------------------------------------
    % Sistema cerrado
    % -----------------------------------------------------

    Acl = Ad-Bd*Kd;

    Bcl = Bd*Kr;

    sys_cl = ss(Acl,Bcl,Cd,Dd,Ts);


    % -----------------------------------------------------
    % Respuesta al escalon
    % -----------------------------------------------------

    [y,t] = step(sys_cl,tiempo);


    y = squeeze(y);


    % -----------------------------------------------------
    % Caracteristicas
    % -----------------------------------------------------

    info = stepinfo(y,t,referencia);


    % -----------------------------------------------------
    % Error estacionario
    % -----------------------------------------------------

    error_estacionario = abs(referencia-y(end));


    % -----------------------------------------------------
    % Guardar resultados
    % -----------------------------------------------------

    resultados(i,1) = i;

    resultados(i,2) = parteReal;

    resultados(i,3) = parteImag;

    resultados(i,4) = K1;

    resultados(i,5) = K2;

    resultados(i,6) = Kr;

    resultados(i,7) = info.RiseTime;

    resultados(i,8) = info.SettlingTime;

    resultados(i,9) = info.Overshoot;

    resultados(i,10) = error_estacionario;


    % -----------------------------------------------------
    % Graficar
    % -----------------------------------------------------

    plot(t,y,'LineWidth',1.5);

end


%% =========================================================
% 8. CONFIGURACION DE LA GRAFICA
% =========================================================

xlabel('Tiempo (s)');

ylabel('Posicion');

title('Comparacion de diferentes polos deseados');

legendText = strings(numeroCasos,1);

for i = 1:numeroCasos

    legendText(i) = sprintf( ...
        'Caso %d: %.2f %+.2fi', ...
        i, ...
        casos(i,1), ...
        casos(i,2));

end

legend(legendText,'Location','best');

hold off;


%% =========================================================
% 9. TABLA DE RESULTADOS
% =========================================================

disp(' ');

disp('=======================================================');

disp('             RESULTADOS DE COMPARACION');

disp('=======================================================');


tabla = array2table( ...
    resultados, ...
    'VariableNames',{ ...
    'Caso', ...
    'Real', ...
    'Imag', ...
    'K1', ...
    'K2', ...
    'Kr', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'ErrorFinal'});


disp(tabla);


%% =========================================================
% 10. SELECCIONAR EL CASO PARA ESP32
% =========================================================

disp(' ');

disp('=======================================================');

disp('          SELECCION DE CONFIGURACION');

disp('=======================================================');

casoElegido = input( ...
    'Ingrese el numero de caso que desea llevar al ESP32: ');


%% =========================================================
% 11. RECALCULAR CASO SELECCIONADO
% =========================================================

parteReal = casos(casoElegido,1);

parteImag = casos(casoElegido,2);


% Polos continuos

p_cont = [
    parteReal + parteImag*1i;
    parteReal - parteImag*1i
];


% Polos discretos

p_disc = exp(p_cont*Ts);


% Controlador

Kd = place(Ad,Bd,p_disc);

K1 = Kd(1);

K2 = Kd(2);


% Kr

Kr = 1/(Cd*((eye(size(Ad))-Ad+Bd*Kd)\Bd));


% Lazo cerrado

Acl = Ad-Bd*Kd;


%% =========================================================
% 12. OBSERVADOR
% =========================================================

% Observador aproximadamente 4 veces mas rapido

p_obs_cont = [
    4*parteReal + 4*parteImag*1i;
    4*parteReal - 4*parteImag*1i
];


p_obs_disc = exp(p_obs_cont*Ts);


Ld = place(Ad',Cd',p_obs_disc)';


L1 = Ld(1);

L2 = Ld(2);


%% =========================================================
% 13. MOSTRAR CONFIGURACION ELEGIDA
% =========================================================

disp(' ');

disp('=======================================================');

disp('             CONFIGURACION SELECCIONADA');

disp('=======================================================');


fprintf('\nCaso seleccionado = %d\n',casoElegido);


disp(' ');

disp('Polos continuos:');

disp(p_cont);


disp('Polos discretos:');

disp(p_disc);


disp(' ');

fprintf('K1 = %.15f\n',K1);

fprintf('K2 = %.15f\n',K2);

fprintf('Kr = %.15f\n',Kr);


disp(' ');

fprintf('L1 = %.15f\n',L1);

fprintf('L2 = %.15f\n',L2);


%% =========================================================
% 14. VALORES COMPLETOS PARA ESP32
% =========================================================

disp(' ');

disp('=======================================================');

disp('              VALORES PARA ESP32');

disp('=======================================================');


fprintf('\nTs = %.15f;\n',Ts);


fprintf('\nAd = [\n');

fprintf('%.15f  %.15f;\n', ...
    Ad(1,1),Ad(1,2));

fprintf('%.15f  %.15f\n', ...
    Ad(2,1),Ad(2,2));

fprintf('];\n');


fprintf('\nBd = [\n');

fprintf('%.15f;\n',Bd(1));

fprintf('%.15f\n',Bd(2));

fprintf('];\n');


fprintf('\nK1 = %.15f;\n',K1);

fprintf('K2 = %.15f;\n',K2);

fprintf('Kr = %.15f;\n',Kr);


fprintf('\nL1 = %.15f;\n',L1);

fprintf('L2 = %.15f;\n',L2);


%% =========================================================
% 15. INFORMACION FINAL
% =========================================================

disp(' ');

disp('=======================================================');

disp('                 FIN DEL ANALISIS');

disp('=======================================================');

