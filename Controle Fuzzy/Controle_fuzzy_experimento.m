% Sistemas Fuzzy - Código 02/02 - Experimento
% Atividade prática - Síntese de um controlador fuzzy projetado para o
% modelo identificado de um sistema subamortecido 
% Aluno: Alan Sovano Gomes

clear all; close all; clc;

% Condições iniciais do experimento
u(1:4) = 0;
u1(1:4) = 0;
u2(1:4) = 0;
y(1:4) = 0;
e(1:4) = 0;
var_erro(1:4) = 0;
y_nc(1:4) = 0;

% Período de amostragem 
Ts = 0.01;

% Número de iterações
N = 2000;

% Sinal de referência
y_ref(1:N) = [ones(1,N/4) 2*ones(1,N/4) 0*ones(1,N/4) 3*ones(1,N/4)]; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUZZY PI %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Carregando o controlador Fuzzy PI projetado
controlador = readfis('Sistema_PI_ou_PD');

% Relações entre ganhos fuzzy e ganho de um PI
%GE*GCU => Ki
%GCE*GCU >= Kp
GE = 0.05;    % Ganho da entrada fuzzy 01
GCE = 0.01;  % Ganho da entrada fuzzy 02 
GCU = 1;  % Ganho da saída fuzzy


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUZZY PD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Carregando o controlador Fuzzy PD projetado 
controlador2 = readfis('Sistema_PI_ou_PD');

% Relações entre ganhos fuzzy e ganho de um PD
%GCE*GU => Kd
%GE*GU => Kp
GE2 = 0.05;    % Ganho da entrada fuzzy 01
GCE2 = 0.3;  % Ganho da entrada fuzzy 02 
GU = 1;  % Ganho da saída fuzzy


% Limites do universo de discurso das variáveis
v1_max = 4;
v1_min = -4;
v2_max = 4;
v2_min = -4;


%%%%%%%%%%%%%%%%%%%%%%% Laço de controle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Iniciando o daqduino (código de aquisição de dados)
% OBS: VERIFICAR A PORTA COM DO ARDUINO!!!
daqduino_start('COM7');

for k=5:N

    % Amostrando a saída
    y(k) = daqduino_read;
    
    
    %%%%%%% Calculo das variáveis de entrada dos sistema fuzzy %%%%%%%%%%%%
    % Para o Fuzzy-PI
    e(k) = GE*(y_ref(k) - y(k));
    var_erro(k) = GCE/GE*(e(k) - e(k-1));
    
    % Para o Fuzzy-PD
    e2(k) = GE2*(y_ref(k) - y(k));
    var_erro2(k) = GCE2/GE2*(e2(k) - e2(k-1));
    
   %%%%%%%%%%%%%% Condições para lidar com saturação %%%%%%%%%%%%%%%%%%%%%% 
   % Limitando a entrada 01 do PI
   if e(k) > v1_max
        er = v1_max;
    elseif e(k)<v1_min
        er = v1_min;   
    else
        er=e(k);
    end
    
    
    % Limitando a entrada 02 do PI
    if var_erro(k)>v2_max
        d_e = v2_max;
    elseif var_erro(k)<v2_min
        d_e = v2_min;    
    else
        d_e=var_erro(k);
    end
    
    
    % Limitando a entrada 01 do PD
     if e2(k) > v1_max
        er2 = v1_max;
    elseif e2(k)<v1_min
        er2 = v1_min;   
    else
        er2=e2(k);
    end
    
    
    % Limitando a entrada 02 do PD
    if var_erro2(k)>v2_max
        d_e2 = v2_max;
    elseif var_erro2(k)<v2_min
        d_e2 = v2_min;    
    else
        d_e2=var_erro2(k);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%% Lei de controle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Saída do controlador fuzzy PI
    du(k) = GCU*evalfis([er d_e],controlador);
    u1(k) = u1(k-1) + du(k);
    
    % Saída do controlador fuzzy PD
    u2(k) = GU*evalfis([er2 d_e2],controlador2);
    
    % Lei de controle PID -> u(k)= u1(k) + u2(k); 
    % Para lei de controle PI -> u(k)= u1(k) + 0*u2(k)
    % Para lei de controle PD -> u(k) = 0*u1(k) + u2(k)
    % Para sistema em malha aberta -> u(k) = y_ref(k)
    u(k)= u1(k) + 0*u2(k); 
    
    daqduino_write(u(k),Ts);
end
daqduino_end;

% Vetor com instantes de tempo
t = 0:Ts:(N-1)*Ts;

% Plotando os resultados
figure(1)
stairs(t, y);
title('Saída do sistema real controlado')
xlabel('Tempo (s)');
ylabel('Amplitude (V)');

% Plotando o sinal de controle
figure(2)
stairs(t, u);
title('Sinal de controle real');
xlabel('Tempo(s)');
ylabel('Amplitude (V)');







