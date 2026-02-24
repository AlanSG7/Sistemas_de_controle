% Sistemas Fuzzy - Código 0/02 - Projeto
% Atividade prática - Síntese de um controlador fuzzy projetado para o
% modelo identificado de um sistema subamortecido 
% Aluno: Alan Sovano Gomes

clear all; close all; clc;

%%%%%%%%%%%%%%%% Descrição do modelo identificado %%%%%%%%%%%%%%%%%%%%%%%%%

% O modelo é referente a um circuito que apresenta uma série de filtros em 
% série, de forma a compor uma saída subamortecida. O sistema foi modelado
% a partir do método dos Mínimos Quadrados Recursivos (MQR), buscando-se
% obter um modelo ARX apropriado.

% Parâmetros sistema identificado 
a1 = -1.90058053047511;
a2 = 0.547798985041972;
a3 = 0.785675695276235;
a4 = -0.395129684464947;
b0 = 0.0193098393940883;
b1 = 0.0182048624549816;

% Tempo de amostragem do sistema
Ts = 0.004;

% Montando a função de transferência do sistema
Gz = tf([b0 b1 0],[1 a1 a2 a3 a4], Ts);

% Avaliando a resposta ao degrau
figure(1)
step(Gz)

% Avalaindo a variação do erro da resposta ao degrau
[y_Gz, t_Gz] = step(Gz);
erro = 1-y_Gz;

% Calculando a variação do erro
for k=2:length(y_Gz)
   
    variacao_erro(k-1) = erro(k) - erro(k-1);
    t_variacao_erro(k-1) = (t_Gz(k) + t_Gz(k-1))/2; 
    
end

% Plotando o erro
figure(2)
stairs(t_Gz, erro);
xlabel('Tempo (s)');
ylabel('Amplitude do erro (V)');
title('Erro do seguimento de referência');

% Plotando o gráfico da variação do erro
figure(3)
stairs(t_variacao_erro, variacao_erro)
xlabel('Tempo (s)');
ylabel('Variação do erro (V/s)');
title('Variação do erro de seguimento de referência');



%%%%%%%%%%%%%%%%%%%%%%%% Sistema de controle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Número de iterações
N = 5000;

% Sinal de referência
y_ref(1:N) = [ones(1,N/4) 2*ones(1,N/4) 0*ones(1,N/4) 3*ones(1,N/4)]; 

% Vetor de tempo de simulação
t = 0:Ts:(N-1)*Ts;

% Condições iniciais
u(1:4) = 0;
u1(1:4) = 0;
u2(1:4) = 0;
y(1:4) = 0;
e(1:4) = 0;
var_erro(1:4) = 0;
y_nc(1:4) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUZZY PI %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Carregando o controlador Fuzzy PI projetado
controlador = readfis('Sistema_PI_ou_PD');

% Relações entre ganhos fuzzy e ganho de um PI
%GE*GCU => Ki
%GCE*GCU => Kp
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


%%%%%%%%%%%%%%%%%%%%%%% Laço de controle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=5:N

    % Equação a diferenças do processo
    y(k) = -a1*y(k-1) - a2*y(k-2) - a3*y(k-3) - a4*y(k-4) +0.7*(b0*u(k-2) +b1*u(k-1));
    
    % Processo não controlado
    y_nc(k) = -a1*y_nc(k-1) - a2*y_nc(k-2) - a3*y_nc(k-3) - a4*y_nc(k-4) +0.7*(b0*y_ref(k-2) +b1*y_ref(k-1));
    
    % Calculo das variáveis de entrada dos sistema fuzzy
    
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
    
    %%%%%%%%%%%%% Fim das condições de saturação %%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
    
    
end

% Plotando os sinais de saída
figure(4)
stairs(t, y);
hold
stairs(t,y_nc);
title('Sinal de saída do sistema em malha aberta e em malha fechada');
xlabel('Tempo (s)');
ylabel('Amplitude (V)');
legend('Sistema com controlador Fuzzy (PI)','Sistema sem controlador');

% Plotando o sinal de controle
figure(5)
stairs(t, u);
title('Sinal de controle');
xlabel('Tempo(s)');
ylabel('Amplitude (V)');



