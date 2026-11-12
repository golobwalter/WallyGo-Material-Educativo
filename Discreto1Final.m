clc;
clear;
close all;

%% =========================================================
%        DISEÑO COMPLETO DEL CONTROLADOR PARA ESP32
% =========================================================
%
% Planta continua:
%
%       x_dot = A*x + B*u
%       y     = C*x + D*u
%
% El ESP32 ejecutará el controlador cada Ts = 10 ms.
%
% El controlador utiliza:
%
%       u(k) = -Kd*xhat(k) + Kr*r(k)
%
% y un observador:
%
%       xhat(k+1) = Ad*xhat(k) + Bd*u(k)
%                  + Ld*(y(k)-Cd*xhat(k))
%
% =========================================================


%% =========================================================
% 1. MODELO CONTINUO
% =========================================================

A = [0      1;
     0   -11.5];

B = [0;
     125];

C = [1 0];

D = 0;


%% =========================================================
% 2. TIEMPO DE MUESTREO
% =========================================================

Ts = 0.01;       % 10 ms


%% =========================================================
% 3. POLOS DESEADOS CONTINUOS
% =========================================================

  p_cont = [-1.3333 + 1.3983i;
          -1.3333 - 1.3983i];
      
%  p_cont = [-1 + 1i;
%          -1 - 1i];
      
%  p_cont = [-2 + 2i;
%          -2 - 2i];


%% =========================================================
% 4. CONTROLADOR CONTINUO
% =========================================================

K_cont = place(A,B,p_cont);

Acl_cont = A - B*K_cont;

polos_cont_obtenidos = eig(Acl_cont);


%% =========================================================
% 5. GANANCIA Kr CONTINUA
% =========================================================

Kr_cont = -1/(C*(Acl_cont\B));


%% =========================================================
% 6. DISCRETIZACION DE LA PLANTA
% =========================================================

sys_c = ss(A,B,C,D);

sys_d = c2d(sys_c,Ts,'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;


%% =========================================================
% 7. POLOS CONTINUOS -> POLOS DISCRETOS
% =========================================================

p_disc = exp(p_cont*Ts);


%% =========================================================
% 8. CONTROLADOR DISCRETO
% =========================================================

Kd = place(Ad,Bd,p_disc);

Acl_d = Ad - Bd*Kd;

polos_d_control = eig(Acl_d);


%% =========================================================
% 9. GANANCIA DE REFERENCIA DISCRETA
% =========================================================

Kr_d = 1/(Cd*((eye(2)-Ad+Bd*Kd)\Bd));


%% =========================================================
% 10. OBSERVADOR
% =========================================================
%
% Elegimos polos del observador aproximadamente 4 veces
% más rápidos que los polos del controlador.
%

p_obs_cont = [-5.3332 + 5.5932i;
              -5.3332 - 5.5932i];


%% =========================================================
% 11. POLOS DEL OBSERVADOR EN DISCRETO
% =========================================================

p_obs_disc = exp(p_obs_cont*Ts);


%% =========================================================
% 12. GANANCIA DEL OBSERVADOR
% =========================================================

Ld = place(Ad',Cd',p_obs_disc)';

polos_observador = eig(Ad-Ld*Cd);


%% =========================================================
% 13. MOSTRAR RESULTADOS
% =========================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('        RESULTADOS DEL DISEÑO DE CONTROL\n');
fprintf('=====================================================\n');


%% MODELO CONTINUO

fprintf('\n--- MODELO CONTINUO ---\n');

fprintf('A =\n');
disp(A);

fprintf('B =\n');
disp(B);

fprintf('C =\n');
disp(C);

fprintf('D =\n');
disp(D);


%% POLOS CONTINUOS

fprintf('\n--- POLOS DESEADOS CONTINUOS ---\n');

fprintf('%.12f %+.12fi\n',real(p_cont(1)),imag(p_cont(1)));
fprintf('%.12f %+.12fi\n',real(p_cont(2)),imag(p_cont(2)));


%% K CONTINUO

fprintf('\n--- CONTROLADOR CONTINUO ---\n');

fprintf('K_cont(1) = %.15f\n',K_cont(1));
fprintf('K_cont(2) = %.15f\n',K_cont(2));

fprintf('Kr_cont   = %.15f\n',Kr_cont);

fprintf('\nPolos obtenidos:\n');

fprintf('%.12f %+.12fi\n', ...
    real(polos_cont_obtenidos(1)), ...
    imag(polos_cont_obtenidos(1)));

fprintf('%.12f %+.12fi\n', ...
    real(polos_cont_obtenidos(2)), ...
    imag(polos_cont_obtenidos(2)));


%% MATRICES DISCRETAS

fprintf('\n--- MATRICES DISCRETAS ---\n');

fprintf('Ts = %.15f segundos\n',Ts);

fprintf('\nAd =\n');
disp(Ad);

fprintf('Bd =\n');
disp(Bd);

fprintf('Cd =\n');
disp(Cd);

fprintf('Dd =\n');
disp(Dd);


%% POLOS DISCRETOS

fprintf('\n--- POLOS DESEADOS DISCRETOS ---\n');

fprintf('%.12f %+.12fi\n',real(p_disc(1)),imag(p_disc(1)));
fprintf('%.12f %+.12fi\n',real(p_disc(2)),imag(p_disc(2)));


%% K DISCRETO

fprintf('\n--- CONTROLADOR DISCRETO ---\n');

fprintf('K1 = %.15f\n',Kd(1));
fprintf('K2 = %.15f\n',Kd(2));

fprintf('Kr = %.15f\n',Kr_d);


%% POLOS CONTROLADOS

fprintf('\nPolos del sistema controlado:\n');

fprintf('%.12f %+.12fi\n', ...
    real(polos_d_control(1)), ...
    imag(polos_d_control(1)));

fprintf('%.12f %+.12fi\n', ...
    real(polos_d_control(2)), ...
    imag(polos_d_control(2)));


%% OBSERVADOR

fprintf('\n--- OBSERVADOR ---\n');

fprintf('Ld1 = %.15f\n',Ld(1));
fprintf('Ld2 = %.15f\n',Ld(2));

fprintf('\nPolos del observador:\n');

fprintf('%.12f %+.12fi\n', ...
    real(polos_observador(1)), ...
    imag(polos_observador(1)));

fprintf('%.12f %+.12fi\n', ...
    real(polos_observador(2)), ...
    imag(polos_observador(2)));


%% =========================================================
% 14. VALORES DEFINITIVOS PARA ESP32
% =========================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('          VALORES PARA EL ESP32\n');
fprintf('=====================================================\n');

fprintf('\nTs = %.15f;\n',Ts);

fprintf('\nAd = [\n');
fprintf('%.15f  %.15f;\n',Ad(1,1),Ad(1,2));
fprintf('%.15f  %.15f\n',Ad(2,1),Ad(2,2));
fprintf('];\n');

fprintf('\nBd = [\n');
fprintf('%.15f;\n',Bd(1));
fprintf('%.15f\n',Bd(2));
fprintf('];\n');

fprintf('\nK1 = %.15f;\n',Kd(1));
fprintf('K2 = %.15f;\n',Kd(2));
fprintf('Kr = %.15f;\n',Kr_d);

fprintf('\nLd1 = %.15f;\n',Ld(1));
fprintf('Ld2 = %.15f;\n',Ld(2));


%% =========================================================
% 15. ECUACION DEL OBSERVADOR
% =========================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('          ECUACION DEL OBSERVADOR\n');
fprintf('=====================================================\n');

fprintf('\n');
fprintf('xhat(k+1) = Ad*xhat(k) + Bd*u(k)\n');
fprintf('            + Ld*(y(k)-Cd*xhat(k))\n');


%% =========================================================
% 16. LEY DE CONTROL
% =========================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('             LEY DE CONTROL\n');
fprintf('=====================================================\n');

fprintf('\n');
fprintf('u(k) = -Kd*xhat(k) + Kr*r(k)\n');


%% =========================================================
% 17. SIMULACION DEL SISTEMA CERRADO
% =========================================================

sys_cl_d = ss(Acl_d,Bd*Kr_d,Cd,Dd,Ts);

figure;

step(sys_cl_d);

grid on;

title('Control de posicion discreto');

xlabel('Tiempo (s)');

ylabel('Posicion');


%% =========================================================
% 18. INFORMACION DEL SISTEMA
% =========================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('             FIN DEL CALCULO\n');
fprintf('=====================================================\n');

fprintf('\nEl ESP32 debe ejecutar el algoritmo cada %.3f ms.\n',Ts*1000);

fprintf('\n');