#include <SparkFunCCS811.h>


#define CCS811_ADDR 0x5B 
CCS811 myCCS811(CCS811_ADDR);
void setup()
{
    Serial.begin(9600);
    myCCS811.begin();

}

void loop()
{
  if (myCCS811.dataAvailable())
  {
    myCCS811.readAlgorithmResults();
    //Returns calculated CO2 reading in parts per million (ppm)
    int tempCO2 = myCCS811.getCO2();
    Serial.print("CO2:");
    Serial.print(tempCO2);
    Serial.print("\t");
    //Returns calculated hithere Voltatile Organic Compounds (TVOC) reading in parts per billion (ppb)
    int tempVOC = myCCS811.getTVOC();
    Serial.print("tempVOC:");
    Serial.print(tempVOC);
    Serial.print("\t");
    float Temp = myCCS811.getTemperature();
    Serial.print("Temperature:");
    Serial.println(Temp);
    
    
  }
 
  delay(1000); //Wait for next reading
}
