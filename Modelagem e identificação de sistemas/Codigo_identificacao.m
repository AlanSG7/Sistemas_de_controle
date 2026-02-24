% Modelagem e Identificação de Sistemas Dinâmicos
% Atividade prática - Identificação de um sistema eletrônico subamortecido
% Aluno: Alan Sovano Gomes

clear all; close all; clc;

%%%%%%%%%%%%%%%%% Avaliação da resposta ao degrau %%%%%%%%%%%%%%%%%%%%%%%%%

y_step = dlmread('step.txt');          % Carregando dados experimentais
y_step = y_step(562:end,:);
Ts = 1/250;                            % Frequência de amostragem de 250 Hz
t_step = 0:Ts:size(y_step,1)*Ts - Ts;  % Vetor com instantes de tempo

% Plotando o step
figure(1)
stairs(t_step, y_step);
title('Step response of the system');
xlabel('Time (s)');
ylabel('Amplitude (V)');

% Avaliando a resposta ao degrau, verifica-se que o período de oscilação
% é de 0.1 s, logo:
Wn = 2*pi*1/0.1;


%%%%%%%%%%%%%%%%%%%%%%%%% Projetando o sinal PRBS %%%%%%%%%%%%%%%%%%%%%%%%%

fmin = Wn/(2*pi*20);            % Frequência da dinâmica mais lenta a ser 
                                % excitada
                                
fmax = 5*Wn/(2*pi);             % Frequência da dinâmica mais rápida a ser 
                                % excitada 

Tb = 1/(3*fmax);           % Tempo entre bits

% Como Ts está entre os limites indicados de Tb (AGUIRRE, 2014), podemos
% fazer:
Tb = Ts;

n=ceil(log2(1/(fmin*Tb))); % Número de bits deve ser 9 (XOR bit 5 e 9)


%%%%%%%%%%%%%%%%%%%%%% Avaliando ensaio experimental %%%%%%%%%%%%%%%%%%%%%%

% Foi feito um ensaio experimental do sistema, onde foi aplicado um PRBS,
% entre 1 e 2, à entrada da planta. 

% Carregando os dados experimentais:
dados = dlmread('ident.txt');
PRBS = dados(:,1);
y_experimental = dados(:,2);

% Gerando vetor de tempo para os dados experimentais
t_experimental = 0:Ts:size(y_experimental,1)*Ts - Ts;

% Tratamento prévio dos dados: remoção de média (evita a polarização do
% estimador por mínimos quadrados.
%PRBS = PRBS - mean(PRBS);
%PRBS = detrend(PRBS);
%y_experimental = y_experimental - mean(y_experimental);
%y_experimental = detrend(y_experimental);

% Plotando a função de autocorrelação do PRBS
figure(2)
autocorr(PRBS,size(PRBS,1)-1);

% Plotando a saída e o PRBS com relação ao tempo
figure(3)
subplot(211)
plot(t_experimental,y_experimental);
title('Input and output signals of the system')
ylabel('Amplitude (V)');
xlabel ('Time (s)')
subplot(212)
plot(t_experimental,PRBS)
ylabel('Amplitude (V)')
xlabel('Time (s)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Divisão dos dados %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dados para validar o modelo:
y_validacao = y_experimental(513:end,:);
u_validacao = PRBS(513:end,:);
t_validacao = 0:Ts:size(y_validacao,1)*Ts - Ts;

% Dados para levantar o modelo:
y_ident = y_experimental(1:512,:);
u_ident = PRBS(1:512,:);



%%%%%%%%%%%%%% Algoritmo dos Mínimos Quadrados Recursivo (MQR) %%%%%%%%%%%%

% Tamanho de parâmetros
num_theta = 7;

% Condições iniciais do algoritmo
THETA_hat = zeros(num_theta, size(u_ident,1));   % Vetor com os valores de THETA_hat 
                                              % para cada iteração
                                              
P = 1000*eye(num_theta,num_theta);            % Valor inicial da matriz de covariância
nit  = size(u_ident, 1);                         % Número de iterações do algoritmo
y_hat = zeros(size(u_ident,1));                  % Vetor de saídas estimadas
PHI = zeros(num_theta,1);                     % Vetor com os regressores (dados)
K = zeros(num_theta,1);                       % Ganho do estimador
TRACOP = zeros(1,size(u_ident,1));               % Traço da matriz de covariância
e_est = zeros(1, size(u_ident,1));                  % Erro de identificação

% Condições iniciais do traço da matriz de covariância
TRACOP(1) = trace(P);
TRACOP(2) = trace(P);
TRACOP(3) = trace(P);

% Erro de estimação para modelar C(z^-1)
%e_est(4) = 

% Algoritmo MQR
for i=5:nit
   
    % Vetor de regressores
    PHI = [-y_ident(i-1) -y_ident(i-2) -y_ident(i-3) -y_ident(i-4) u_ident(i-2) u_ident(i-3)]';
    
    % Calculando a saída estimada a partir do parâmetros estimados
    y_hat(i)= PHI'*THETA_hat(:,i-1);
    
    % Erro de estimação
    e_est(i) = y_ident(i) - y_hat(i);
    
    % Ganho do estimador 
    K = P*PHI/(1 + PHI'*P*PHI);
    
    % Atualização do vetor de parâmetros estimados
    THETA_hat(:,i) = THETA_hat(:,i-1) + K*e_est(i);
    
    % Atualização da matriz de covariância
    P = P - K*(P*PHI)';
    TRACOP(i) = trace(P);
   
end

% Plotando gráfico do erro de identificação
figure(4)
plot(abs(e_est));
xlabel('Iterations');
ylabel('Error');
title('Absolute identification error')

% Plotando gráficos dos parâmetros identificados ao longo do tempo
figure(5)
plot(THETA_hat(1,:));
hold
plot(THETA_hat(2,:));
plot(THETA_hat(3,:));
plot(THETA_hat(4,:));
plot(THETA_hat(5,:));
plot(THETA_hat(6,:));
xlabel('Iterations');
ylabel('Parameters Values');
legend('a1','a2', 'a3', 'a4', 'b0', 'b1');
title('Evolution of the parameters along iterations');

% Plotando o gráfico do traço da matriz de covariância
figure(6)
plot(TRACOP);
xlabel('Iterações');
ylabel('Magnitude de tr(P)')
title('Traço da matriz de covariância');

%%%%%%%%%%%%%%%%%%%%%%%% Comparando os modelos %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Contruindo o modelo identificado
a1 = THETA_hat(1,end);
a2 = THETA_hat(2,end);
a3 = THETA_hat(3,end);
a4 = THETA_hat(4,end);
b0 = THETA_hat(5,end);
b1 = THETA_hat(6,end);
sys_identificado = tf([b0 b1 0],[1 a1 a2 a3 a4], Ts);


% Comparando os resultados (01)
figure(7)
lsim(sys_identificado, u_validacao, t_validacao,1.5);
hold
stairs(t_validacao,y_validacao, 'r');

legend('Identified system', 'Experimental data');
title('Comparison between the outputs to for a PRBS input');
xlabel('Time');
ylabel('Amplitude (V)');

% Comparando os resultados (02)
figure(8)
step(sys_identificado,t_step);
hold;
stairs(t_step,y_step, 'r');
legend('Identified system', 'Experimental data')
title('Comparison (system identification using RLS estimator)');
xlabel('Time');
ylabel('Amplitude (V)');


% Calculando R^2
Y_ident = step(sys_identificado, t_step);
SEQ = sumsqr((Y_ident-y_step));
R_quad = 1 - SEQ/sumsqr(y_step - mean(y_step));




%%%%%%%%%%%%%%%%%%%%%% Modelagem por degrau (2 ordem) %%%%%%%%%%%%%%%%%%%%%

% Organizando os dados para modelagem
dados_ident_step = [t_step', ones(size(y_step)), y_step];

% Modelando por resposta ao degrau
[num, den, parametros] = modelar(dados_ident_step);
G = tf(num, den);

% Comparando os resultados 
figure(9)
step(G,t_step);
hold;
stairs(t_step,y_step, 'r');
legend('Modeled system', 'Experimental data')
title('Comparison (modeling using step response)');
xlabel('Time');
ylabel('Amplitude (V)');



%%%%%%%%%%%%%%%%%%%%%%% Modelagem fenomenologica %%%%%%%%%%%%%%%%%%%%%%%%%%


% Parâmetros do circuito em estudo:
C1 = 1.0*10^-6;
C2 = 1.0*10^-9;
R1 = 470000;
R2 = 470000;
C = 1.0*10^-6;
R = 6800;

% Montando a função de transferência (filtro 2 ordem):
num = 1/(R1*R2*C1*C2);
den = [1 (R1+R2)/(C1*R1*R2) 1/(C1*C2*R1*R2)];
Sallen_Key = tf(num, den);

% Montando a função de transferência (filtro 1 ordem):
num2 = 1/(R*C);
den2 = [1, 1/(R*C)];
filtro_RC = tf(num2, den2);

H = Sallen_Key*filtro_RC;

% Comparando os resultados 
figure(10)
step(H,t_step);
hold;
stairs(t_step,y_step, 'r');
legend('Modeled system', 'experimental data')
title('Comparison (phenomenological modeling)');
xlabel('Time');
ylabel('Amplitude (V)');







