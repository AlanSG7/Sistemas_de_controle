// Frequência de amostragem em 500 Hz (definida a partir do timer)
#define INTERVAL_LENGTH_US 2000UL  

// defines para dar set ou clear nos bits do registrador 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Variáveis utilizadas no controle da amostragem e no proceso de identificação
unsigned long previousMicros;
int PRBS[9] = {0, 0, 0, 0, 1, 0, 0, 0, 1}; // Valor inicial do PRBS
int atualiza;


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
    // Variável para o loop for de atualização do PRBS
    int i;

    // Loop de controle de amostragem. Caso o tempo de amostragem seja menor que 0.004 s, o processamento é interrompido
    while((micros() - previousMicros) <= INTERVAL_LENGTH_US) 
    {
       unsigned long currentMicros = micros();
       previousMicros += INTERVAL_LENGTH_US;

      // Atualização do PRBS
       atualiza = PRBS[4]^PRBS[8];
       for(i=8; i>0;i--)
       {
        PRBS[i]=PRBS[i-1];
       }
       PRBS[0]=atualiza;

       // Enviando o comando para o sistema
       analogWrite(10, PRBS[8]*255/5  + 255/5);

       // Plotando a entrada e a saída
       Serial.print(PRBS[8] + 1);
       Serial.print(',');    
       Serial.print((double)analogRead(A0)*5/1023, 2); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).
       Serial.print(',');    
       Serial.println((double)analogRead(A1)*5/1023, 2); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).
    } 
}
