// Frequência de amostragem em 250 Hz (definida a partir do timer)
#define INTERVAL_LENGTH_US 2000UL  

// defines para dar set ou clear nos bits do registrador 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Variáveis utilizadas para o sistema de controle (erro, entrada, saída e referência)
unsigned long previousMicros;
char comando;
float yr[3] = {0.0, 0.0, 0.0};
float y[5] = {0.0, 0.0, 0.0, 0.0, 0.0} ;
float u[2] = {0.0, 0.0};
float du[3] = {0.0, 0.0, 0.0};

// Parâmetros do controlador contínuo
const float Kp = 0.01;
const float Ki = 4;
const float Kd = 0.001;

// Passando o controlador contínuo para o domínio discreto através de uma aproximação backward
const float Ts = 0.004;
const float s0 = Kp + Ki*Ts + Kd/Ts;
const float s1 = -Kp - 2*Kd/Ts;
const float s2 = Kd/Ts;

// Parâmetros para o controlador PID com augmentção estocástica (calculados a partir da equação diofantina -> código feito no MATLAB)
const float f[5] = {0.417040589375850, -0.777055678393674, 0.256784774024964, 0.234039285572753, -0.114808970579893};
const float BECQ[3] = {0.015329515672768, 0.0106352133438838,  0.00528960895670294}; // primeiro elemento = 0.005329515672768 + lambda

// Ruído aleatório
double randNumber;

void setup()
{
  // Prescaler definido para 64 (clock de 250kHz) -> clock = 16MHz/prescaler (default do arduino é prescaler=128)
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Começando a comunicação serial
  Serial.begin(115200);
  pinMode(10,OUTPUT);
  analogWrite(10, 0);

  // Gerador de ruído
  randomSeed(analogRead(A1));
  
}


void loop()
{

    int i;

    // Loop de controle de amostragem. Caso o tempo de amostragem seja menor que 0.004 s, o processamento é interrompido
    while((micros() - previousMicros) <= INTERVAL_LENGTH_US) 
    {
       unsigned long currentMicros = micros();
       previousMicros += INTERVAL_LENGTH_US;


       while(Serial.available())
       {
        comando = Serial.read();
        if(comando=='s')
         {
           yr[0] = 0.5;

      } 

         else if(comando=='r')
          {
           yr[0] = 0.75;

          }
          
         else if(comando=='t')
          {
           yr[0] = 1.0;

          }
           else if(comando=='q')
          {
           yr[0] = 0.0;
          }
 
       }
           randNumber = random(-50,51)/2000.0;
           y[0] =  (double)analogRead(A0)*5/1023;
           du[0] = (1/BECQ[0])*(-BECQ[1]*du[1] -BECQ[2]*du[2] +s0*yr[0] +s1*yr[1] +s2*yr[2] -f[0]*y[0] -f[1]*y[1] -f[2]*y[2] -f[3]*y[3] -f[4]*y[4]);
           u[0] = u[1] +du[0];


           
           if(u[0]<0)
           {
            u[0]=0;
            }

            
           if(u[0]>5)
           {
            u[0]=5;
            }

          // Enviando o comando para o sistema
          analogWrite(10, u[0]*255/5);


          // Deslocando os valores do sinal de referência, saída, controle e incremento de controle nas memórias
          du[2] = du[1];
          du[1] = du[0];
          
          yr[2] = yr[1];
          yr[1] = yr[0];

          y[4] = y[3];
          y[3] = y[2];
          y[2] = y[1];
          y[1] = y[0];
          
          u[1] = u[0];

         // Plotando a entrada e a saída
         Serial.print(u[0],4);
         Serial.print(',');    
         Serial.println(y[0], 4); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).

    } 
}
