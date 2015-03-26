#include <Esplora.h>
#include <TFT.h>
#include <SPI.h>
int x;
int y;
int vectorY;
void setup()
{
  Serial.begin(9600);
  EsploraTFT.begin();
  EsploraTFT.background(0,0,0);
  
}
static void sendPosition(){

  //  Serial.print(x);
  Serial.print(Esplora.readJoystickX());
  Serial.print("#");
  Serial.print(Esplora.readJoystickY());
  Serial.print("#");
  Serial.print(Esplora.readJoystickButton());
  Serial.print("#");
  Serial.print(Esplora.readButton(SWITCH_DOWN));
  Serial.print("#");
  Serial.print(Esplora.readButton(SWITCH_LEFT));
  Serial.print("#");
  Serial.print(Esplora.readButton(SWITCH_UP));
  Serial.print("#");
  Serial.print(Esplora.readButton(SWITCH_RIGHT));
  Serial.print("\n");
  
}
static void sendTemperature()
{
  Serial.print(Esplora.readTemperature(DEGREES_C));
}
void loop()
{
  
  sendPosition();
  //  sendLightSensor();
  //  send
  delay(10);
}
