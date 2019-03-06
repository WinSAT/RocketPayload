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
    //Returns calculated Total Voltatile Organic Compounds (TVOC) reading in parts per billion (ppb)
    int tempVOC = myCCS811.getTVOC();
    float myCCS811.getTemperature();
    Serial.print(tempCO2);Serial.print(tempVOC);
  }
  else if (myCCS811.checkForStatusError())
  {
    Serial.print("error");
  }

  delay(1000); //Wait for next reading
}
