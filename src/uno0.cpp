#include <Arduino.h>
#include <math.h>

#define pinPWM 6      //Configura o pino de Saida do PWM
#define PERIODO 100   //Em ms 100ms

byte count = 0;

void setup() // Codigo de configuração
{
  pinMode(pinPWM, OUTPUT);
}

#define TIME_RESOLUTION 1
uint32_t previousTimeMS = 0;
void loop()
{
  const uint32_t currentTimeMS = millis();
  if ((currentTimeMS - previousTimeMS) >= TIME_RESOLUTION)
  {
    previousTimeMS = currentTimeMS;
    analogWrite(pinPWM, (uint8_t)(120.0 * (sin(2 * PI * count / PERIODO) + 1.1)));
    count = (count + 1) % PERIODO;  
  }
}
