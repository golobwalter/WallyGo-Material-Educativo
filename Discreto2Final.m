
clc;
clear;
close all;

%% =========================================================
%      PROGRAMA DE DISEÑO Y PRUEBAS - CONTROL ESP32
%
% ENTRADAS:
%   A, B, C, D
%   Polos deseados continuos
%   Ts
%
% CALCULA AUTOMATICAMENTE:
%   Ad, Bd, Cd, Dd
%   Polos discretos
%   Kd
%   Kr
%   Ld
%
% TAMBIEN REALIZA:
%   Simulacion de respuesta al escalon
%   Caracteristicas de la respuesta
%   Valores listos para ESP32
%
% =========================================================


%% =========================================================
% 1. DATOS DE LA PLANTA
% =========================================================

% ---------------------------------------------------------
% MOTOR / PLANTA ACTUAL
% ---------------------------------------------------------

A = [0      1;
     0   -11.5];

B = [0;
     125];

C = [1 0];

D = 0;


% Tiempo de muestreo del ESP32

Ts = 0.01;       % 10 ms


%% =========================================================
% 2. POLOS DESEADOS CONTINUOS
% =========================================================

% ---------------------------------------------------------
% MODIFICAR ESTOS DOS VALORES PARA PROBAR OTROS POLOS
% ---------------------------------------------------------

parteReal = -1.3333;  % entrar el nuevo valor real deseado

parteImag =  1.3983;  % entrar el nuevo valor Imag deseado


% Se genera automaticamente el par de polos conjugados

p_cont = [parteReal + parteImag*1i;
          parteReal - parteImag*1i];


%% =========================================================
% 3. MOSTRAR DATOS DE ENTRADA
% =========================================================

disp(' ');
disp('=====================================================');
disp('              DATOS DE DISEÑO');
disp('=====================================================');

disp(' ');

disp('Matriz A =');
disp(A);

disp('Matriz B =');
disp(B);

disp('Matriz C =');
disp(C);

disp('Matriz D =');
disp(D);

fprintf('\nTiempo de muestreo Ts = %.6f segundos\n', Ts);

disp(' ');

disp('Polos deseados continuos =');
disp(p_cont);


%% =========================================================
% 4. DISCRETIZACION DE LA PLANTA
% =========================================================

sys_c = ss(A,B,C,D);

sys_d = c2d(sys_c,Ts,'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;


disp(' ');
disp('=====================================================');
disp('              MODELO DISCRETO');
disp('=====================================================');

disp(' ');

disp('Ad =');
disp(Ad);

disp('Bd =');
disp(Bd);

disp('Cd =');
disp(Cd);

disp('Dd =');
disp(Dd);


%% =========================================================
% 5. CONVERSION DE POLOS CONTINUOS A DISCRETOS
% =========================================================

p_disc = exp(p_cont*Ts);


disp(' ');
disp('Polos deseados discretos =');

disp(p_disc);


%% =========================================================
% 6. VERIFICACION DE CONTROLABILIDAD
% =========================================================

Co = ctrb(Ad,Bd);

rangoControlabilidad = rank(Co);

disp(' ');

if rangoControlabilidad == size(Ad,1)

    disp('Sistema CONTROLABLE: OK');

else

    disp('ATENCION: EL SISTEMA NO ES CONTROLABLE');

end


%% =========================================================
% 7. CALCULO DEL CONTROLADOR DISCRETO
% =========================================================

Kd = place(Ad,Bd,p_disc);

Acl = Ad - Bd*Kd;

polos_control = eig(Acl);


disp(' ');
disp('=====================================================');
disp('             CONTROLADOR DISCRETO');
disp('=====================================================');

disp(' ');

disp('Kd =');
disp(Kd);

fprintf('\nK1 = %.15f\n', Kd(1));

fprintf('K2 = %.15f\n', Kd(2));

disp(' ');

disp('Polos obtenidos en lazo cerrado =');

disp(polos_control);


%% =========================================================
% 8. GANANCIA DE REFERENCIA Kr
% =========================================================

% Ley de control:
%
% u(k) = -Kd*xhat(k) + Kr*r(k)
%
% Se calcula Kr para obtener ganancia estatica unitaria.

Kr = 1/(Cd*((eye(size(Ad))-Ad+Bd*Kd)\Bd));


disp(' ');
disp('Ganancia de referencia Kr =');

disp(Kr);


%% =========================================================
% 9. DISEÑO DEL OBSERVADOR
% =========================================================

% ---------------------------------------------------------
% El observador se diseña 4 veces mas rapido
% que el controlador.
% ---------------------------------------------------------

parteRealObs = 4*parteReal;

parteImagObs = 4*parteImag;


p_obs_cont = [parteRealObs + parteImagObs*1i;
              parteRealObs - parteImagObs*1i];


% Convertir polos del observador a discreto

p_obs_disc = exp(p_obs_cont*Ts);


% Calcular ganancia del observador

Ld = place(Ad',Cd',p_obs_disc)';


% Polos reales del observador

polos_observador = eig(Ad-Ld*Cd);


disp(' ');
disp('=====================================================');
disp('                  OBSERVADOR');
disp('=====================================================');

disp(' ');

disp('Polos observador continuos =');

disp(p_obs_cont);

disp(' ');

disp('Polos observador discretos =');

disp(p_obs_disc);

disp(' ');

disp('Ld =');

disp(Ld);

fprintf('\nL1 = %.15f\n', Ld(1));

fprintf('L2 = %.15f\n', Ld(2));

disp(' ');

disp('Polos obtenidos del observador =');

disp(polos_observador);


%% =========================================================
% 10. SISTEMA EN LAZO CERRADO
% =========================================================

Bcl = Bd*Kr;

sys_cl = ss(Acl,Bcl,Cd,Dd,Ts);


%% =========================================================
% 11. RESPUESTA A ESCALON
% =========================================================

figure;

step(sys_cl);

grid on;

title('Control de posicion - Sistema discreto');

xlabel('Tiempo (s)');

ylabel('Posicion');


%% =========================================================
% 12. CARACTERISTICAS DE LA RESPUESTA
% =========================================================

info = stepinfo(sys_cl);


disp(' ');
disp('=====================================================');
disp('          CARACTERISTICAS DE RESPUESTA');
disp('=====================================================');

fprintf('\nTiempo de establecimiento = %.6f segundos\n', ...
        info.SettlingTime);

fprintf('Tiempo de subida = %.6f segundos\n', ...
        info.RiseTime);

fprintf('Sobreimpulso = %.3f %%\n', ...
        info.Overshoot);


%% =========================================================
% 13. VALORES COMPLETOS PARA EL ESP32
% =========================================================

disp(' ');
disp('=====================================================');
disp('             VALORES PARA EL ESP32');
disp('=====================================================');

fprintf('\nTs = %.15f;\n', Ts);


fprintf('\nAd = [\n');

fprintf('%.15f  %.15f;\n', ...
        Ad(1,1),Ad(1,2));

fprintf('%.15f  %.15f\n', ...
        Ad(2,1),Ad(2,2));

fprintf('];\n');


fprintf('\nBd = [\n');

fprintf('%.15f;\n', Bd(1));

fprintf('%.15f\n', Bd(2));

fprintf('];\n');


fprintf('\nCd = [\n');

fprintf('%.15f  %.15f\n', ...
        Cd(1),Cd(2));

fprintf('];\n');


fprintf('\nDd = %.15f;\n', Dd);


fprintf('\nK1 = %.15f;\n', Kd(1));

fprintf('K2 = %.15f;\n', Kd(2));

fprintf('Kr = %.15f;\n', Kr);


fprintf('\nL1 = %.15f;\n', Ld(1));

fprintf('L2 = %.15f;\n', Ld(2));


%% =========================================================
% 14. ECUACION DEL OBSERVADOR
% =========================================================

disp(' ');
disp('=====================================================');
disp('              ECUACION DEL OBSERVADOR');
disp('=====================================================');

disp(' ');

disp('xhat(k+1) = Ad*xhat(k) + Bd*u(k)');

disp('             + Ld*(y(k)-Cd*xhat(k))');


%% =========================================================
% 15. LEY DE CONTROL
% =========================================================

disp(' ');
disp('=====================================================');
disp('                 LEY DE CONTROL');
disp('=====================================================');

disp(' ');

disp('u(k) = -Kd*xhat(k) + Kr*r(k)');


%% =========================================================
% 16. RESUMEN FINAL
% =========================================================

disp(' ');
disp('=====================================================');
disp('                RESUMEN FINAL');
disp('=====================================================');

fprintf('\n');

disp('Polos deseados continuos:');

for k = 1:length(p_cont)

    fprintf('%.10f %+.10fi\n', ...
        real(p_cont(k)), ...
        imag(p_cont(k)));

end


disp(' ');

disp('Polos deseados discretos:');

for k = 1:length(p_disc)

    fprintf('%.10f %+.10fi\n', ...
        real(p_disc(k)), ...
        imag(p_disc(k)));

end


fprintf('\nK1 = %.12f\n', Kd(1));

fprintf('K2 = %.12f\n', Kd(2));

fprintf('Kr = %.12f\n', Kr);

fprintf('\nL1 = %.12f\n', Ld(1));

fprintf('L2 = %.12f\n', Ld(2));


disp(' ');
disp('=====================================================');
disp('               FIN DEL PROGRAMA');
disp('=====================================================');

