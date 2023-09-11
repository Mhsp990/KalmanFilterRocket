#include "Bmp_library_rocket.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define altvar 0.05 //Variação Mínima para detectar queda



int funcao_altitude_bmp() //As configurações de todos os BMPs serão feitas no setup do código principal.
{

  float alt1=0,alt2=0; // alt2 será o valor anterior(primeira leitura), enquanto alt1 será o valor atual(segunda leitura).
  int counter;
  
  for(counter=0; (!bmp.begin())&& (counter <40); counter++) //Enquanto não detectar o bmp E contador< valor,print na tela.
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  if(counter>40) return 3; //Se não detectou o bmp durante o limite estipulado, retorne 3 para o vetor a fim de diminuir o impacto na análise do vetor.

    //Printar na saída os valores lidos para facilitar analise dos dados
    Serial.println("Primeira Leitura");
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    //Primeiro Cálculo de Altitude.
    alt2=bmp.readAltitude(1014);//Atualizando o valor atual. Utilizar valor lido através de um site local. Link no documento do drive.
    Serial.print(F("Approx altitude = "));
    Serial.print(alt2); /* Adjusted to local forecast! */
    Serial.println(" m");
    delay(100); //Cria-se uma pequena diferença de tempo entre dois valores para realização dos testes.Verificar frequência de amostragem caso necessário para otimizar..


    //Segundo Cálculo de Altitude , ignora-se a temperatura desta vez.
    Serial.println("Segunda Leitura");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

     alt1=bmp.readAltitude(1014);//Atualizando o valor atual
    Serial.print(F("Approx altitude = "));
    Serial.print(alt1); /* Adjusted to local forecast! */
    Serial.println(" m");
    Serial.println();

    //Comparando altitudes: Valor atual com o valor anterior
    if(alt1!=0 && alt2!=0 && (alt2 > (alt1+altvar)) ) //Queda de altitude maior do que o tolerado para evitar erros de leituras
    {
      
      return 1;// Retorna "1" na função, que indica que uma queda foi detectada neste sensor.
      
      Serial.println("QUEDA DETECTADA");
       Serial.println( "alt1=");
       Serial.print(alt1);
       Serial.println();
       Serial.println( "alt2=");
       Serial.print(alt2);
       Serial.println();
      Serial.println();
      
    }
    

return 0; //Não foi detectada uma queda nesse ciclo de amostragem.
  
}
