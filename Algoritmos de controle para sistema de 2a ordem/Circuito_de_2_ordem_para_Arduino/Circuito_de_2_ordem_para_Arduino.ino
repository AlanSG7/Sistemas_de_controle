// Frequência de amostragem em 250 Hz (definida a partir do timer)
#define INTERVAL_LENGTH_US 4000UL  

// defines para dar set ou clear nos bits do registrador 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

unsigned long previousMicros;
char comando;


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
    
    while((micros() - previousMicros) <= INTERVAL_LENGTH_US) // Vê se o tempo entre duas amostras é igual ou menor ao periodo de amostragem
    {
       unsigned long currentMicros = micros();
       previousMicros += INTERVAL_LENGTH_US;

       // Plotando os dados na porta serial. O tempo está em segundos e o valor de leitura do conversor A/D está sendo convertido para valores de tensão (V=leitura*5/1023)
       //Serial.println(((double)currentMicros)/1000000UL, 3); // 3 is the number of decimals to print
       //Serial.print(',');
       Serial.println((double)analogRead(A0)*5/1023, 4); // Mostra o valor de tensão lido com 4 casas decimais (o arduino consegue diferenciar valores de tensão que tem uma diferença de aprox. 4.9 mV).

       while(Serial.available())
       {
        comando = Serial.read();
        if(comando=='s')
         {
           analogWrite(10, 255*3/5);
         }
         else if(comando=='r')
          {
            analogWrite(10,0);
          }
 
       }


}
}
