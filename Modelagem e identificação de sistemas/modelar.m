% Modelagem e Identificação de Sistemas Dinâmicos
% Trabalho 01 - Modelagem de sistemas de 1a e 2a ordem por resposta ao
% degrau.
% Aluno: Alan Sovano Gomes
% Código 1/2 - Função "modelar.m"

% A função "modelar.m" gera o modelo de um sistema de 1ª ordem (sem zeros) 
% ou de 2ª ordem (subamortecido) a partir de dados experimentais. A função
% avalia se a saída do sistema apresenta overshoot para tomar a decisão de
% qual modelo escolher (quando há overshoot, o modelo é de 2ª ordem, caso
% contrário, de 1ª ordem).

% Para utilizr a função construida, é necessário passar como argumento
% uma matriz contendo um vetor com instantes de tempo na primeira coluna,
% o sinal de entrada (do tipo degrau) na segunda coluna e o sinal de saída
% na terceira coluna.

% A função "modelar.m" irá retornar 3 vetores linha: o primeiro contendo o
% numerador da função de transferência do modelo, o segundo contendo o
% denominador e o terceiro contendo os parâmetros encontrados para o
% sistema.

% Caso o sistema seja modelado por uma função de tranferência de 1ª ordem,
% o vetor de parâmetros retornado conterá o ganho estático do sistema e a
% constante de tempo, respectivamente. Caso o modelo seja de segunda ordem,
% os parâmetros retornados serão o ganho estático do sistema, a frequência
% natural e o fator de amortecimento.

function [num, den, parametros] = modelar(dados)

% Separando os dados em vetores contendo os instantes de tempo, o sinal de
% entrada e o sinal de saída:
t = dados(:,1);
u = dados(:,2);
y = dados(:,3);

% Variável contendo o tamanho dos vetores:
L = length(t);

% Encontrando o ganho estático da planta:
Kdc = y(end)/u(end);

% Verificando se o sistema é de primeira ou segunda ordem:

if ((max(y)/Kdc)>u(end)) 
    % Se houver algum valor de y maior que a entrada u (com essa entrada 
    % sendo do tipo degrau), então o sistema é de 2ª ordem e subamortecido. 
    
    % O valor de y é corrigdo pelo ganho estático da planta dentro do teste
    % condicional (de forma a não classificar um sistema de 1ª ordem com
    % ganho maior que 1 como um sistema de segunda ordem, por exemplo).
    
    % Encontrando o pico do sinal de saída (considerando um ganho estático
    % unitário), o tempo de pico e a amplitude do degrau aplicado:
    
    [pico_saida_Kdc, indice_tp] = max(y);
    tp = t(indice_tp);
    amplitude_degrau = u(end);
     pico_saida = pico_saida_Kdc/(Kdc*amplitude_degrau);
    
    % calculando o coeficiente de amortecimento:
    k = log(pico_saida-1);
    csi = sqrt(k^2/(k^2+pi^2));
    
    % Calculando a frequência natural do sitema:
    Wn = pi/(tp*sqrt(1-csi^2));
    
    % Montando o modelo:
    num = Kdc*Wn^2;
    den = [1 2*csi*Wn Wn^2];
    parametros = [Kdc Wn csi];

       
else
    % Se o sistema não apresentar sobressinal, então ele será modelado como
    % um sistema de 1ª ordem.
    
    % Como o ganho estático do sistema já foi calculado, só resta encontrar
    % a constante de tempo do modelo de 1ª ordem. Encontrando a posição do
    % vetor de tempo que corresponde à constante de tempo:
    
    for i =1:L
        if(y(i)>=0.632*y(end))
            % Encontrando o instante onde y é 63,2% do valor final. 
            break
        end
    end
    
    tau = t(i);
    
    % Montando o modelo:
    
    num = Kdc;
    den = [tau 1];
    parametros = [Kdc tau];
    
end
    
end





