// Frequência de amostragem em 250 Hz (definida a partir do timer)
#define INTERVAL_LENGTH_US 4000UL  

// defines para dar set ou clear nos bits do registrador 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Variáveis utilizadas para o sistema de controle (erro, entrada, saída e referência)
unsigned long previousMicros;
char comando;
float yr = 0.0;
float y = 0.0;
float u[2] = {0, 0};
float e[3] = {0, 0, 0};

// Parâmetros do controlador contínuo
const float Kp = 0.01;
const float Ki = 4;
const float Kd = 0.001;

// Passando o controlador contínuo para o domínio discreto através de uma aproximação backward
const float Ts = 0.004;
const float s0 = Kp + Ki*Ts + Kd/Ts;
const float s1 = -Kp - 2*Kd/Ts;
const float s2 = Kd/Ts;

// Ruído aleatório
double randNumber = 0;


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
           yr = 1.0;

      } 

         else if(comando=='r')
          {
           yr = 0.0;

          }
 
       }
           randNumber = random(-50,51)/2000.0;
           y =(double)analogRead(A0)*5/1023;
           e[0] = yr - y;
           u[0] = 0*randNumber +  u[1] +s0*e[0] +s1*e[1] +s2*e[2];

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


          // Deslocando os valores do sinal de erro e de controle nas memórias
          e[2] = e[1];
          e[1] = e[0];
          u[1] = u[0];

         // Plotando a entrada e a saída
         Serial.print(u[0],4);
         Serial.print(',');    
         Serial.println(y, 4); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).

    } 
}
