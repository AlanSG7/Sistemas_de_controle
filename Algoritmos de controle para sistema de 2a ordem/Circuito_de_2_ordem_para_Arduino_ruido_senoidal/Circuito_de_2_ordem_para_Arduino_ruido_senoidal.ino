// Frequência de amostragem em 250 Hz (definida a partir do timer)
#define INTERVAL_LENGTH_US 4000UL  

// defines para dar set ou clear nos bits do registrador 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Variáveis utilizadas para o sistema de controle (erro, entrada, saída e referência)
unsigned long previousMicros;
char comando;
float y = 0.0;
float u1[2] = {0.0, 0.0};
float yr = 0.0;
float du = 0.0;
float e[3]={0.0, 0.0, 0.0};
float u2[3]={0.0, 0.0, 0.0};
float u = 0.0;

// Parâmetros do controlador PID contínuo
const float Kp = 0.01;
const float Ki = 15;
const float Kd = 0.005;

// Passando o controlador PID contínuo para o domínio discreto através de uma aproximação backward
const float Ts = 0.004;
const float s0 = Kp + Ki*Ts + Kd/Ts;
const float s1 = -Kp - 2*Kd/Ts;
const float s2 = Kd/Ts;

// Controlador proporcional-ressonante sintonizado na frequência de 1 rad/s

const float a1_c = -1.99837400541110;
const float a2_c = 0.999973344351002;
const float b0_c = 0.000413162559475417;
const float b1_c = 0.000799669469952420;
const float b2_c = 0.000386506910477003;


//ruido senoidal
const float omega = 10;
int k=0;


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
           yr = 0.0;

      } 

         else if(comando=='r')
          {
           yr = 1.0;

          }
          
         else if(comando=='t')
          {
           yr = 2.0;

          }
 
       }
           y =  (double)analogRead(A0)*5/1023 + (0.1*sin(omega*k*0.004));
           e[0] = yr - y;
           du = s0*e[0] + s1*e[1] + s2*e[2];
           u1[0] = u1[1] + du;
           u2[0] = -a1_c*u2[1] -a2_c*u2[2] + b0_c*e[0] + b1_c*e[1] + b2_c*e[2];
           u = u1[0] + u2[0];


           
           if(u<0)
           {
            u=0;
            }

            
           if(u>5)
           {
            u=5;
            }

          // Enviando o comando para o sistema
          analogWrite(10, u*255/5);


          // Deslocando os valores do sinal de referência, saída, controle e incremento de controle nas memórias
          e[2] = e[1];
          e[1] = e[0];

          u2[2] = u2[1];
          u2[1] = u2[0];

          u1[1] = u1[0];

         // Plotando a entrada e a saída
         Serial.print(u,4);
         Serial.print(',');    
         Serial.println(y, 4); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).

         k=k+1;

    } 
}
